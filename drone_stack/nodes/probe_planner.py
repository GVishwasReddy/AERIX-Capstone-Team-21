"""Where should the aircraft listen next to tell WHICH person holds the phone?

Requirement (2026-10-08): with 2+ people below and the phone not yet placed
on one of them, identify before declining - and get there with as few wrong
handovers and as few declines as possible.

This is not phone localisation. The camera has already said where the
candidates are; the question is a discrete one ("is it person 1, 2 or 3?"),
and the right vantage point is the one where the candidates' predicted RSSI
differ most - not the one where the signal is strongest (prior art steers
toward the strongest signal) and not a ring fixed in advance.

The radio model is PhoneLocator's, so the planner and the estimator agree on
what a measurement is worth. A bin of samples heard from one aircraft position
reads

    r_k = A - L(k, phone) + e_k,   e_k ~ N(0, s_k^2),
    s_k^2 = shadow^2 + noise^2 / n_k,          A ~ N(ref_dbm, ref_sigma^2)

with A the phone's unknown 1 m level, shared by every bin. Under hypotheses i
and j the RSSI vectors are Gaussians with the same covariance
Sigma = diag(s^2) + ref_sigma^2 * 11^T and means differing by
delta = L(., j) - L(., i). Their separation is the Mahalanobis distance

    D_ij^2 = delta^T Sigma^-1 delta
           = sum w delta^2 - ref_sigma^2 (sum w delta)^2 / (1 + ref_sigma^2 sum w)

(Sherman-Morrison, w = 1/s^2), and the Bhattacharyya bound limits the chance
of confusing them to sqrt(p_i p_j) * exp(-D_ij^2 / 8). Summed over pairs that
is ``confusion_bound``; ``next_vantage`` picks the reachable point that
shrinks it most per second of flight.

Two consequences the fixed ring cannot have:
  * a common offset adds nothing (A absorbs it), so only CONTRASTS between
    vantage points count. The planner therefore goes where the candidates'
    loss curves cross and diverge - typically just past one person, away from
    the other - instead of spending dwell time where every hypothesis
    predicts the same thing;
  * shadowing is shared within a bin, so a second dwell on the same spot is
    worth little. The planner sees that and moves on.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RadioModel:
    path_loss_n: float = 2.0
    shadow_sigma_db: float = 4.5
    rssi_sigma_db: float = 6.0
    ref_sigma_db: float = 10.0
    phone_height_m: float = 1.2

    @classmethod
    def of(cls, locator) -> "RadioModel":
        """The model a PhoneLocator is scoring with - keep the two in step."""
        return cls(path_loss_n=locator.path_loss_n,
                   shadow_sigma_db=locator.shadow_sigma_db,
                   rssi_sigma_db=locator.rssi_sigma_db,
                   ref_sigma_db=locator.ref_sigma_db,
                   phone_height_m=locator.phone_height_m)


def _loss(model: RadioModel, bins_ena: np.ndarray, cands: np.ndarray) -> np.ndarray:
    """Path loss (dB) from each bin position (K, 3: east, north, alt) to each
    candidate (N, 2). Returns (N, K)."""
    dh = np.maximum(bins_ena[:, 2] - model.phone_height_m, 0.3)
    d2 = ((cands[:, None, 0] - bins_ena[None, :, 0]) ** 2
          + (cands[:, None, 1] - bins_ena[None, :, 1]) ** 2 + dh[None, :] ** 2)
    return 5.0 * model.path_loss_n * np.log10(np.maximum(d2, 0.09))


def separation(model: RadioModel, bins_ena: np.ndarray, counts: np.ndarray,
               cands: np.ndarray) -> np.ndarray:
    """D_ij^2 for every pair of candidates, given bins heard from. (N, N)."""
    n = len(cands)
    out = np.zeros((n, n))
    if len(bins_ena) == 0:
        return out
    w = 1.0 / (model.shadow_sigma_db ** 2
               + model.rssi_sigma_db ** 2 / np.maximum(counts, 1))
    va = model.ref_sigma_db ** 2
    loss = _loss(model, bins_ena, cands)
    for i in range(n):
        for j in range(i + 1, n):
            d = loss[j] - loss[i]
            sw, swd = (w * d * d).sum(), (w * d).sum()
            out[i, j] = out[j, i] = sw - va * swd * swd / (1.0 + va * w.sum())
    return out


def confusion_bound(probs, d2: np.ndarray) -> float:
    """Upper bound on P(the top pick is the wrong person). 0 = certain."""
    p = np.asarray(probs, dtype=float)
    total = 0.0
    for i in range(len(p)):
        for j in range(i + 1, len(p)):
            total += math.sqrt(max(p[i] * p[j], 0.0)) * math.exp(-d2[i, j] / 8.0)
    return total


@dataclass
class Vantage:
    east: float
    north: float
    bound_after: float      # predicted confusion bound after listening here
    gain_per_s: float       # bound reduction per second of flight + dwell


def next_vantage(model: RadioModel, cands, probs, bins, feasible,
                 here: tuple[float, float], alt: float, *, dwell_s: float = 2.0,
                 rate_hz: float = 4.0, speed_mps: float = 1.0,
                 bin_m: float = 0.75) -> Vantage | None:
    """The feasible point whose RSSI best separates the candidates still in
    contention, per second it costs to get there and listen.

    cands     [(east, north)] people the camera sees
    probs     current P(holds the phone) per candidate (PhoneLocator)
    bins      [(east, north, alt, mean_rssi, n)] already heard (PhoneLocator.bins)
    feasible  [(east, north)] points the caller has cleared for flight
    """
    if len(cands) < 2 or not feasible:
        return None
    c = np.asarray(cands, dtype=float).reshape(-1, 2)
    b = np.asarray([(e, n, a) for e, n, a, _, _ in bins], dtype=float).reshape(-1, 3)
    k = np.asarray([cnt for *_, cnt in bins], dtype=float)
    now = confusion_bound(probs, separation(model, b, k, c))
    n_new = max(1.0, dwell_s * rate_hz)
    best: Vantage | None = None
    for e, n in feasible:
        # Listening inside a bin already heard from adds samples to THAT bin
        # (shared shadowing: worth little); elsewhere it opens a new one.
        same = (np.abs(b[:, 0] - e) < bin_m / 2) & (np.abs(b[:, 1] - n) < bin_m / 2) \
            if len(b) else np.zeros(0, dtype=bool)
        if same.any():
            k2 = k.copy()
            k2[np.argmax(same)] += n_new
            b2 = b
        else:
            b2 = np.vstack([b, [e, n, alt]]) if len(b) else np.array([[e, n, alt]])
            k2 = np.append(k, n_new)
        after = confusion_bound(probs, separation(model, b2, k2, c))
        cost = dwell_s + math.hypot(e - here[0], n - here[1]) / max(speed_mps, 0.1)
        gain = (now - after) / cost
        if best is None or gain > best.gain_per_s:
            best = Vantage(float(e), float(n), after, gain)
    return best


def feasible_points(cands, centre: tuple[float, float], max_offset_m: float,
                    *, step_m: float = 1.0, beyond_m: float = 1.5,
                    min_from_person_m: float = 0.0) -> list[tuple[float, float]]:
    """Candidate vantage points: a lattice over the group plus a point just
    past each person, all within ``max_offset_m`` of ``centre`` (the order's
    drop point). Obstacle clearance is the caller's job."""
    c = np.asarray(cands, dtype=float).reshape(-1, 2)
    gx, gy = c.mean(axis=0)
    span = float(np.max(np.hypot(c[:, 0] - gx, c[:, 1] - gy))) + beyond_m + step_m
    pts = []
    ax = np.arange(-span, span + 1e-9, step_m)
    for dx in ax:
        for dy in ax:
            pts.append((gx + dx, gy + dy))
    for px, py in c:
        ux, uy = px - gx, py - gy
        r = math.hypot(ux, uy)
        if r > 0.3:
            pts.append((px + beyond_m * ux / r, py + beyond_m * uy / r))
    out = []
    for e, n in pts:
        if math.hypot(e - centre[0], n - centre[1]) > max_offset_m + 1e-9:
            continue
        if min_from_person_m > 0 and np.min(np.hypot(c[:, 0] - e, c[:, 1] - n)) < min_from_person_m:
            continue
        out.append((float(e), float(n)))
    return out
