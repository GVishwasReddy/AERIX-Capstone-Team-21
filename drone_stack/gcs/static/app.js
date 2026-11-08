/* Drone GCS front-end. Single WebSocket -> render every panel. */
"use strict";

let ws = null, latest = {}, netLatency = 0, seeded = false, wpMode = false;
// Set when the payload slider is wired; called from render() to adopt the
// servo envelope the hub sends. Null until then, and a no-op afterwards once
// the config has been taken once.
let srvAdoptCfg = null;
// Same contract for the independent AUX6 (SERVO14) MG90S buttons.
let aux2AdoptCfg = null;
let altAdoptState = null;        // flight-altitude slider <- /api/state (set in wire())
let altAckHook = null;           // flight-altitude slider <- set_flight_altitude reply
// Same contract again for the flight-recording badge and replay overlay.
let recAdopt = null;
let localWps = [];            // [{lat,lon,alt}]  (map-editable mission)
let lastConsoleId = 0;
let seededConsole = false;    // suppress toasts for the console backlog on first load
let radarMax = 12;            // radar range in metres (zoomable)
let dlvTargetMarker = null, dlvRouteLine = null;
let lastDlvPhase = null;      // for phase-change toasts (null = not yet seeded)
let lastDlvOutcome = null;    // for verdict-change toasts (null = not yet seeded)
let cursorLatLng = null;      // last map cursor position, for "Test order"

/* ---------- NL command history (Up/Down recall, like a shell) ---------- */
const NL_HISTORY_KEY = "aerix_nl_history", NL_HISTORY_MAX = 50;
let nlHistory = [];
try { nlHistory = JSON.parse(localStorage.getItem(NL_HISTORY_KEY)) || []; } catch { nlHistory = []; }
let nlHistoryIdx = nlHistory.length;  // one past the newest entry == "not browsing"
let nlDraft = "";                     // what the user was typing before they pressed Up

/* ---------- WebSocket ----------
   Frames are COALESCED, not rendered on arrival. The browser queues every
   WebSocket message for the main thread with no backpressure of its own, so
   rendering each one meant a page slower than 15 Hz fell further behind every
   second - every button, readout and camera paint waiting behind a queue that
   only a refresh emptied. Now arrivals just replace `pendingFrame`, one render
   runs per animation frame with the newest data, and the ack it sends back is
   what lets the Pi send the next frame (credit window, gcs/ws_flow.py).

   Two keys are deltas, so coalescing must MERGE rather than drop them:
   console lines are sent once each, and the trail only when it changes. */
let pendingFrame = null, pendingConsole = [], renderQueued = false;
let lastTrail = [], trailDirty = false, lastBoot = null;
/* Dead-link watchdog (2026-09-27). In flight the Wi-Fi drops out without the
   socket ever closing: no FIN arrives, so onclose never fires, and the page
   sat showing the last frame as if it were live until TCP gave up (tens of
   seconds). The Pi sends SOMETHING at least every 2 s even to a page that
   stopped acking (ws_flow.FrameGate stall), and pongs every 2 s, so 5 s of
   silence means the connection is dead - drop it and dial a fresh one. */
let lastMsgAt = performance.now(), reconnects = 0, reopenCams = null;
const WS_DEAD_MS = 5000, WS_STALE_MS = 1500;
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const sock = new WebSocket(`${proto}://${location.host}/ws`);
  ws = sock;
  lastMsgAt = performance.now();
  sock.onclose = () => { if (ws === sock) setTimeout(connect, 1000); };
  sock.onmessage = (e) => {
    if (ws !== sock) return;
    lastMsgAt = performance.now();
    const d = JSON.parse(e.data);
    d._rx = lastMsgAt;
    if (d.pong !== undefined) { netLatency = Math.round(performance.now() - d.pong); return; }
    if (d.ack !== undefined) { onAck(d.ack, d.result); return; }
    // A restarted GCS numbers its console from 1 again; forget our high-water
    // mark or every new line would be discarded as already seen.
    if (d.boot !== undefined && d.boot !== lastBoot) {
      if (lastBoot !== null) lastConsoleId = 0;
      lastBoot = d.boot;
    }
    if (d.console && d.console.length) {
      pendingConsole.push(...d.console);
      if (pendingConsole.length > 200) pendingConsole.splice(0, pendingConsole.length - 200);
    }
    if (d.trail !== undefined) { lastTrail = d.trail; trailDirty = true; }
    latest = d; pendingFrame = d;
    if (!renderQueued) { renderQueued = true; requestAnimationFrame(flushFrame); }
  };
}
function flushFrame() {
  renderQueued = false;
  const d = pendingFrame;
  if (!d) return;
  pendingFrame = null;
  d.console = pendingConsole; pendingConsole = [];
  d.trail = lastTrail;
  try { render(d); }
  finally {
    if (d.seq !== undefined && ws && ws.readyState === 1) {
      // hold = how long THIS page sat on the frame (waiting for an animation
      // frame, rendering). The Pi subtracts it so the round trip it measures
      // is the network's, and a slow laptop is not mistaken for a congested
      // link that should cost the video.
      const hold = Math.round(performance.now() - (d._rx || performance.now()));
      ws.send(JSON.stringify({ cmd: "frame_ack", seq: d.seq, hold }));
    }
  }
}
function wsWatchdog() {
  const age = performance.now() - lastMsgAt;
  const pill = $("pill-gcs");
  if (pill && age > WS_STALE_MS) {
    // Say so the moment the numbers on screen stop being live - a frozen
    // altitude that LOOKS current is worse than one marked stale.
    setPill(pill, `STALE ${(age / 1000).toFixed(1)}s`, age > WS_DEAD_MS ? "bad" : "warn");
  }
  // A hidden tab's timers are throttled and it is not being flown from.
  if (document.hidden || age < WS_DEAD_MS || !ws) return;
  if (ws.readyState === 0 && age < 2 * WS_DEAD_MS) return;   // still dialling
  reconnects += 1;
  const dead = ws;
  ws = null;                       // its onclose must not dial a second socket
  try { dead.close(); } catch (_) { /* already gone */ }
  connect();
  if (typeof reopenCams === "function") reopenCams();   // same dead path
}
setInterval(wsWatchdog, 500);
function send(cmd, params) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd, params: params || {} }));
}
function onAck(cmd, res) {
  if (cmd === "set_flight_altitude" && altAckHook) altAckHook(res);
  if (cmd === "nl") {
    nlLog((res.ok ? "✓ " : "✗ ") + (res.message || "no response"), res.ok);
    const actions = (res.data && res.data.actions) || [];
    actions.forEach((a) => nlLog("   • " + (a.intent || "") + " → " + a.message, a.ok));
  }
  if (cmd === "record") {
    $("btn-record").textContent = res.recording ? "Stop Recording" : "Record Log";
    $("btn-record").classList.toggle("danger", !!res.recording);
    if (!res.recording) loadLogList();
  }
}
setInterval(() => { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd: "ping", t: performance.now() })); }, 2000);

/* ---------- helpers ---------- */
const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => (x === undefined || x === null) ? "--" : Number(x).toFixed(d);
function setPill(el, text, cls) { el.textContent = text; el.className = "pill " + (cls || ""); }

/* innerHTML only when the markup actually changed. Several panels were torn
   down and rebuilt 15 times a second with identical content: wasted layout,
   garbage for the collector, and - worse - lost clicks. A click needs press
   and release on the SAME element; rebuild the row between the two and the
   click never fires, which reads to the operator as a laggy, ignored button.
   So a region is also left alone while a pointer is held down inside it; the
   next frame after release catches it up. */
let heldTarget = null;
document.addEventListener("pointerdown", (e) => { heldTarget = e.target; }, true);
["pointerup", "pointercancel"].forEach((t) =>
  window.addEventListener(t, () => { heldTarget = null; }, true));
function setHtml(el, html) {
  if (!el || el._html === html) return;
  if (heldTarget && el.contains(heldTarget)) return;
  el._html = html;
  el.innerHTML = html;
}

/* ---------- translucent centre-screen announcement ---------- */
let lastOverride = null;
let lastDropStage = null;
let lastMode = null;         // last flight mode we announced (null = not yet seeded)
let _toastTimer = null;
function showToast(text, cls) {
  const el = $("mode-toast");
  if (!el) return;
  el.textContent = text;
  el.className = "mode-toast " + (cls || "");
  void el.offsetWidth;                       // restart the fade transition
  el.classList.add("show");
  clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => el.classList.remove("show"), 2600);
}

/* ---------- theme ---------- */
function initTheme() {
  const saved = localStorage.getItem("gcs-theme") || "dark";
  document.body.dataset.theme = saved;
  $("theme-toggle").textContent = saved === "dark" ? "☀" : "☽";
  $("theme-toggle").onclick = () => {
    const next = document.body.dataset.theme === "dark" ? "light" : "dark";
    document.body.dataset.theme = next;
    localStorage.setItem("gcs-theme", next);
    sparkColor = null;
    $("theme-toggle").textContent = next === "dark" ? "☀" : "☽";
  };
}

/* ---------- UI density ----------
   The whole stylesheet is sized off a single scale factor (--sc), so one data
   attribute on <body> rescales every font, control height and grid track at
   once. Operators run this on anything from a 13" laptop to a bench monitor;
   let them pick rather than guessing a fixed size for all of them. */
const DENSITIES = ["compact", "normal", "roomy"];
function initDensity() {
  const btn = $("density-toggle");
  if (!btn) return;
  const apply = (d) => {
    // :root, not body - the metric scale is declared on :root and only
    // re-substitutes var(--dsc) if --dsc is set on that same element.
    document.documentElement.dataset.density = d;
    btn.textContent = d === "compact" ? "\u25AB" : d === "roomy" ? "\u25A0" : "\u25AA";
    btn.title = "UI density: " + d + " (click to change)";
  };
  let cur = localStorage.getItem("gcs-density") || "normal";
  if (DENSITIES.indexOf(cur) < 0) cur = "normal";
  apply(cur);
  btn.onclick = () => {
    cur = DENSITIES[(DENSITIES.indexOf(cur) + 1) % DENSITIES.length];
    localStorage.setItem("gcs-density", cur);
    apply(cur);
    // Leaflet caches the container size; a density change resizes it.
    setTimeout(() => { try { map && map.invalidateSize(); } catch (e) {} }, 60);
  };
}

/* ---------- viewport ----------
   Leaflet caches its container size. Without this the map keeps whatever
   dimensions it had at load, so after any window resize, tablet rotation or
   density change the tiles sit wrong and the view looks shrunken. This is the
   real fix for that - no amount of CSS reaches it. */
let _rszTimer = null;
function onViewportChange() {
  clearTimeout(_rszTimer);
  _rszTimer = setTimeout(() => {
    try { map && map.invalidateSize(); } catch (e) {}
    // the radar canvas is a fixed 360x360 scaled by CSS and redraws at 15 Hz,
    // so it needs nothing here
  }, 120);
}
window.addEventListener("resize", onViewportChange);
window.addEventListener("orientationchange", onViewportChange);

/* ---------- flight modes ---------- */
const MODES = [
  ["STABILIZE", "STABILIZE"], ["ALT HOLD", "ALT_HOLD"], ["LOITER", "LOITER"], ["POSHOLD", "POSHOLD"],
  ["GUIDED", "GUIDED"], ["AUTO", "AUTO"], ["RTL", "RTL"], ["SMART RTL", "SMART_RTL"],
  ["LAND", "LAND"], ["BRAKE", "BRAKE"], ["CIRCLE", "CIRCLE"], ["ACRO", "ACRO"],
  ["AUTOROTATE", "AUTOROTATE"], ["AUTOTUNE", "AUTOTUNE"], ["AUTO RTL", "AUTO_RTL"], ["AVOID ADSB", "AVOID_ADSB"],
  ["DRIFT", "DRIFT"], ["FLIP", "FLIP"], ["FLOWHOLD", "FLOWHOLD"], ["FOLLOW", "FOLLOW"],
  ["GUIDED NOGPS", "GUIDED_NOGPS"], ["OF LOITER", "OF_LOITER"], ["POSITION", "POSITION"], ["SPORT", "SPORT"],
  ["SYSTEMID", "SYSTEMID"], ["THROW", "THROW"], ["ZIGZAG", "ZIGZAG"],
];
function buildModeGrid() {
  const g = $("mode-grid");
  MODES.forEach(([label, mode]) => {
    const b = document.createElement("button");
    b.textContent = label; b.dataset.mode = mode;
    b.onclick = () => send("set_mode", { mode });
    g.appendChild(b);
  });
}

/* ---------- lidar field of view ---------- */
// The LiDAR's front face is the 0-degree reference: straight up on the radar,
// pointing at the nose of the airframe. `fov_deg` (250) is the arc actually
// scanned - 125 deg left + 125 deg right - and the rest, the wedge directly
// behind, is blanked in the driver so nothing back there ever reaches obstacle
// avoidance. Bearings here are the radar's own convention: 0 = front,
// positive clockwise (to the right).
const DEFAULT_LIDAR_FOV = { enabled: true, fov_deg: 250, half_deg: 125, blind_deg: 110 };

function fovRay(cx, cy, bearing, r) {
  const rad = (bearing - 90) * Math.PI / 180;
  return [cx + Math.cos(rad) * r, cy + Math.sin(rad) * r];
}

/* ---------- radar palette ----------
   A canvas cannot see CSS custom properties, so the radar pulls the theme
   tokens once per theme and caches them. Before this every colour was a
   hardcoded dark-theme hex and the light theme drew a dark-theme radar. */
let _radarPal = null;
function hexA(hex, a) {
  const h = String(hex).replace("#", "");
  if (h.length !== 6) return hex;
  const n = parseInt(h, 16);
  return `rgba(${n >> 16},${(n >> 8) & 255},${n & 255},${a})`;
}
function radarPalette() {
  const theme = document.body.dataset.theme || "dark";
  if (_radarPal && _radarPal.theme === theme) return _radarPal;
  const cs = getComputedStyle(document.body);
  const v = (n, fb) => cs.getPropertyValue(n).trim() || fb;
  const light = theme === "light";
  const cyan = v("--cyan", "#35d6f2"), red = v("--red", "#ff4f60");
  // The dark set keeps the light theme's CONTRAST, not its colours: a slate
  // face (not black) so the free-space fill reads as a solid area, walls as
  // continuous lines, and the rear mask as a visible tint.
  _radarPal = {
    theme, cyan, red,
    green: v("--green", "#3ddc84"), amber: v("--amber", "#ffb340"),
    txt: v("--txt", "#e6ebf0"), dim: v("--dim", "#8b939d"),
    mono: v("--mono", "monospace"),
    face0: light ? "#ffffff" : "#15283a",
    face1: light ? "#e6eef3" : "#0a1520",
    halo:  light ? "#ffffff" : "#0b1621",
    ring:      light ? "rgba(20,60,90,0.14)" : "rgba(150,200,240,0.13)",
    ringMajor: light ? "rgba(20,60,90,0.28)" : "rgba(150,200,240,0.26)",
    spoke:     light ? "rgba(20,60,90,0.07)" : "rgba(150,200,240,0.07)",
    rim:       light ? "rgba(20,60,90,0.50)" : "rgba(160,205,240,0.50)",
    free0:     light ? hexA(cyan, 0.20) : "rgba(70,160,235,0.34)",
    free1:     light ? hexA(cyan, 0.05) : "rgba(70,160,235,0.18)",
    contour:   light ? hexA(cyan, 0.45) : "rgba(125,205,255,0.90)",
    contourW:  light ? 1 : 1.7,
    mask:      light ? hexA(red, 0.06) : hexA(red, 0.13),
    hatch:     light ? hexA(red, 0.11) : hexA(red, 0.20),
  };
  return _radarPal;
}

// Colour of one LiDAR return, banded against the configured critical distance
// (avoidance.stop_distance_m, from the hub) so the picture and the brake agree.
// No stop distance in the frame -> no bands; never invent one here (§5).
function returnColour(pal, r, stop) {
  if (!(stop > 0)) return pal.green;
  if (r <= stop) return pal.red;
  if (r <= stop * 2) return pal.amber;
  if (r <= stop * 4) return pal.green;
  return pal.cyan;
}

// Text with a thin halo in the face colour: readable over the scan without a
// filled box that would hide returns behind the label.
function haloText(ctx, pal, text, x, y, k) {
  ctx.lineWidth = 3 * k; ctx.lineJoin = "round";
  ctx.strokeStyle = pal.halo; ctx.strokeText(text, x, y);
  ctx.fillText(text, x, y);
}

// Bezel: rim, 5-degree ticks, signed bearing labels. Everything here sits
// OUTSIDE the disc, so it can never land on top of a return.
function drawBezel(ctx, cx, cy, R, k, pal, font) {
  ctx.save();
  ctx.strokeStyle = pal.rim; ctx.lineWidth = 1.2 * k;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();
  for (let b = 5; b < 360; b += 5) {
    const major = b % 30 === 0, mid = b % 10 === 0;
    const len = (major ? 6.5 : (mid ? 4 : 2.2)) * k;
    const [x0, y0] = fovRay(cx, cy, b, R + 2 * k);
    const [x1, y1] = fovRay(cx, cy, b, R + 2 * k + len);
    ctx.strokeStyle = major ? pal.rim : hexA(pal.dim, 0.35);
    ctx.lineWidth = (major ? 1.3 : 0.9) * k;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  }
  ctx.font = font(8, 500); ctx.fillStyle = pal.dim;
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (let b = 30; b < 360; b += 30) {
    const s = b <= 180 ? b : b - 360;
    const [x, y] = fovRay(cx, cy, b, R + 17 * k);
    ctx.fillText((s > 0 && s < 180 ? "+" : "") + s, x, y);
  }
  // nose marker: arrow on the rim + FRONT, both outside the disc
  ctx.fillStyle = pal.green;
  ctx.beginPath();
  ctx.moveTo(cx, cy - R - 2 * k);
  ctx.lineTo(cx - 4.5 * k, cy - R - 9.5 * k);
  ctx.lineTo(cx + 4.5 * k, cy - R - 9.5 * k);
  ctx.closePath(); ctx.fill();
  ctx.font = font(8, 700);
  ctx.fillText("FRONT", cx, cy - R - 17 * k);
  ctx.restore();
}

// Ignored rear wedge + the two FOV boundary spokes. Drawn UNDER the returns.
function drawFovMask(ctx, cx, cy, R, fov, k, pal, font) {
  if (!fov.enabled || !(fov.blind_deg > 0.01)) return;
  const half = fov.half_deg;
  const a0 = (half - 90) * Math.PI / 180;          // +125 deg -> back-right
  const a1 = (360 - half - 90) * Math.PI / 180;    // -125 deg -> back-left

  ctx.save();
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, R, a0, a1);
  ctx.closePath();
  ctx.fillStyle = pal.mask;
  ctx.fill();
  ctx.clip();
  ctx.strokeStyle = pal.hatch;
  ctx.lineWidth = 1 * k;
  for (let q = -2 * R; q < 2 * R; q += 7 * k) {    // 45-deg hatching
    ctx.beginPath(); ctx.moveTo(cx + q, cy - R); ctx.lineTo(cx + q + 2 * R, cy + R); ctx.stroke();
  }
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = hexA(pal.red, 0.55);
  ctx.lineWidth = 1.2 * k;
  ctx.setLineDash([4 * k, 4 * k]);
  [half, 360 - half].forEach((b) => {
    const [x, y] = fovRay(cx, cy, b, R);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
  });
  ctx.restore();

  // FOV edge labels, pulled inside the ring so they clear the bezel labels.
  ctx.save();
  ctx.font = font(8.5, 600);
  ctx.fillStyle = hexA(pal.red, 0.9);
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  const [rx, ry] = fovRay(cx, cy, half, R * 0.66);
  const [lx, ly] = fovRay(cx, cy, -half, R * 0.66);
  haloText(ctx, pal, "+" + fmt(half, 0) + "°", rx, ry, k);
  haloText(ctx, pal, "-" + fmt(half, 0) + "°", lx, ly, k);
  ctx.restore();
}

// Scanned arc on the rim + the 0-degree front reference line. Drawn UNDER the
// returns (this used to be a 2.5 px line with a text label drawn OVER them,
// straight up the one bearing that matters most). Thin and translucent, so a
// return dead ahead always shows on top of it.
function drawFovFront(ctx, cx, cy, R, fov, k, pal) {
  const half = fov.enabled ? fov.half_deg : 180;
  ctx.save();
  ctx.strokeStyle = hexA(pal.green, 0.55);
  ctx.lineWidth = 2.2 * k;
  ctx.beginPath();
  ctx.arc(cx, cy, R, (-half - 90) * Math.PI / 180, (half - 90) * Math.PI / 180);
  ctx.stroke();
  ctx.strokeStyle = hexA(pal.green, 0.35);
  ctx.lineWidth = 1.1 * k;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - R); ctx.stroke();
  ctx.restore();
}

// Top-down quad silhouette, nose up. Drawn BEFORE the returns and translucent:
// at 12 m range the icon spans ~0.6 m, and a return that close must not hide.
function drawQuadIcon(ctx, cx, cy, k, pal) {
  const arm = 7.5 * k, rot = 3.6 * k;
  ctx.save(); ctx.translate(cx, cy);
  ctx.strokeStyle = hexA(pal.txt, 0.55); ctx.lineWidth = 1.6 * k;
  [45, 135, 225, 315].forEach((deg) => {
    const [x, y] = fovRay(0, 0, deg, arm);
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(x, y); ctx.stroke();
    ctx.fillStyle = hexA(pal.txt, 0.10);
    ctx.beginPath(); ctx.arc(x, y, rot, 0, 2 * Math.PI); ctx.fill();
    ctx.lineWidth = 1 * k; ctx.stroke(); ctx.lineWidth = 1.6 * k;
  });
  ctx.fillStyle = hexA(pal.txt, 0.85);
  ctx.fillRect(-2.6 * k, -2.6 * k, 5.2 * k, 5.2 * k);
  ctx.fillStyle = pal.green;
  ctx.beginPath();
  ctx.moveTo(0, -arm - rot - 3 * k);
  ctx.lineTo(-2.6 * k, -arm - rot + 1 * k);
  ctx.lineTo(2.6 * k, -arm - rot + 1 * k);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

function avoidColour(pal, status) {
  const s = String(status || "").toUpperCase();
  if (/BRAKE|STOP|EMERG|CRIT/.test(s)) return pal.red;
  if (/SLOW|AVOID|DODGE|STEER|HOLD/.test(s)) return pal.amber;
  if (!s || s === "--" || /STANDBY|OFF|DISABL|IDLE/.test(s)) return pal.dim;
  return pal.green;
}

/* ---------- radar ----------
   Layer order is the design. Everything decorative is drawn first, and the
   raw returns come after it, so nothing added for looks can hide one:
     face -> bezel -> rings/spokes -> rear mask -> stop zone -> free space
     -> front ref -> obstacle halos -> airframe -> contour -> RETURNS
     -> obstacle rings -> corner readouts (outside the disc)
   No animation, no sweep - see the note in style.css. */
function drawRadar(d) {
  const c = $("radar"), ctx = c.getContext("2d");
  const W = c.width, H = c.height, cx = W / 2, cy = H / 2;
  const k = W / 360, pal = radarPalette();
  const maxR = radarMax, R = Math.min(cx, cy) - 30 * k, scale = R / maxR;
  const font = (px, wt) => `${wt || 500} ${px * k}px ${pal.mono}`;
  const av = d.avoidance || {};
  const stop = av.stop_distance_m > 0 ? av.stop_distance_m : null;
  const fov = d.lidar_fov || DEFAULT_LIDAR_FOV;
  const TAU = 2 * Math.PI;
  ctx.clearRect(0, 0, W, H);

  // scope face
  const face = ctx.createRadialGradient(cx, cy, 0, cx, cy, R);
  face.addColorStop(0, pal.face0); face.addColorStop(1, pal.face1);
  ctx.fillStyle = face;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, TAU); ctx.fill();

  drawBezel(ctx, cx, cy, R, k, pal, font);

  // spokes every 30 deg, then range rings (every other one solid)
  ctx.strokeStyle = pal.spoke; ctx.lineWidth = 1 * k;
  for (let b = 0; b < 360; b += 30) {
    const [x, y] = fovRay(cx, cy, b, R);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
  }
  const step = maxR <= 6 ? 1 : (maxR <= 12 ? 2 : (maxR <= 24 ? 4 : 8));
  for (let m = step; m <= maxR + 1e-6; m += step) {
    const major = Math.round(m / step) % 2 === 0;
    ctx.strokeStyle = major ? pal.ringMajor : pal.ring;
    ctx.setLineDash(major ? [] : [2 * k, 4 * k]);
    ctx.beginPath(); ctx.arc(cx, cy, m * scale, 0, TAU); ctx.stroke();
  }
  ctx.setLineDash([]);

  drawFovMask(ctx, cx, cy, R, fov, k, pal, font);

  // ring labels ride the rear bearing: the driver masks that wedge, so no
  // return can ever sit under a label there
  ctx.font = font(7.5, 500); ctx.fillStyle = pal.dim;
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (let m = step; m <= maxR + 1e-6; m += step) {
    if (m * scale < 12 * k) continue;
    haloText(ctx, pal, m + "m", cx, cy + m * scale, k);
  }

  // critical-distance zone, straight from avoidance.stop_distance_m
  if (stop && stop * scale > 2 * k) {
    const rs = Math.min(stop * scale, R);
    ctx.fillStyle = hexA(pal.red, 0.07);
    ctx.beginPath(); ctx.arc(cx, cy, rs, 0, TAU); ctx.fill();
    ctx.strokeStyle = hexA(pal.red, 0.7); ctx.lineWidth = 1.2 * k;
    ctx.setLineDash([3 * k, 3 * k]);
    ctx.beginPath(); ctx.arc(cx, cy, rs, 0, TAU); ctx.stroke();
    ctx.setLineDash([]);
    if (rs > 16 * k) {
      const [sx, sy] = fovRay(cx, cy, 150, rs);
      ctx.font = font(7.5, 700); ctx.fillStyle = pal.red;
      haloText(ctx, pal, "STOP", sx, sy, k);
    }
  }

  // project every valid return once; null keeps the gaps
  const toScreen = (fwd, left) => [cx - left * scale, cy - fwd * scale];
  const scan = d.scan, pts = [];
  if (scan && scan.ranges) {
    const a0 = scan.angle_min, inc = scan.angle_increment;
    for (let i = 0; i < scan.ranges.length; i++) {
      const r = scan.ranges[i];
      if (r === null || !isFinite(r) || r <= 0) { pts.push(null); continue; }
      const a = a0 + i * inc;
      const [x, y] = toScreen(r * Math.cos(a), r * Math.sin(a));
      pts.push({ x, y, r });
    }
  }

  // measured free space: a fan from the airframe to each run of returns.
  // It is the scan itself, filled in - and it sits under the points.
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, TAU); ctx.clip();
  const fs = ctx.createRadialGradient(cx, cy, 0, cx, cy, R);
  fs.addColorStop(0, pal.free0); fs.addColorStop(1, pal.free1);
  ctx.fillStyle = fs;
  ctx.beginPath();
  let open = false;
  for (const p of pts) {
    if (!p) { open = false; continue; }
    if (!open) { ctx.moveTo(cx, cy); open = true; }
    ctx.lineTo(p.x, p.y);
  }
  ctx.fill();
  ctx.restore();

  drawFovFront(ctx, cx, cy, R, fov, k, pal);

  const obs = (d.obstacles || []).slice().sort((a, b) => a.distance - b.distance);
  const nearest = obs.length ? obs[0].distance : null;
  const obsCol = (o) => (o.danger ? pal.red : pal.amber);
  const onDisc = (x, y) => Math.hypot(x - cx, y - cy) <= R + 4 * k;

  // obstacle halos (under the returns)
  obs.forEach((o) => {
    const [px, py] = toScreen(o.x, o.y);
    if (!onDisc(px, py)) return;
    const g = ctx.createRadialGradient(px, py, 0, px, py, 12 * k);
    g.addColorStop(0, hexA(obsCol(o), 0.30)); g.addColorStop(1, hexA(obsCol(o), 0));
    ctx.fillStyle = g;
    ctx.beginPath(); ctx.arc(px, py, 12 * k, 0, TAU); ctx.fill();
  });

  // leader from the airframe to the nearest obstacle
  if (obs.length) {
    const [px, py] = toScreen(obs[0].x, obs[0].y);
    if (onDisc(px, py)) {
      ctx.strokeStyle = hexA(obsCol(obs[0]), 0.6); ctx.lineWidth = 1 * k;
      ctx.setLineDash([2 * k, 3 * k]);
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(px, py); ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  drawQuadIcon(ctx, cx, cy, k, pal);

  // contour between neighbouring returns - only across small range steps, so
  // a depth jump reads as an edge instead of a wall that is not there
  ctx.strokeStyle = pal.contour; ctx.lineWidth = pal.contourW * k;
  ctx.beginPath();
  for (let i = 1; i < pts.length; i++) {
    const p = pts[i - 1], q = pts[i];
    if (!p || !q || Math.abs(p.r - q.r) > 0.4 || p.r > maxR || q.r > maxR) continue;
    ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y);
  }
  ctx.stroke();

  // THE RETURNS - top data layer. Beyond the zoom range they are pinned to
  // the rim as dim ticks rather than vanishing off-canvas.
  for (const p of pts) {
    if (!p) continue;
    if (p.r > maxR) {
      const u = R - 2 * k, dx = p.x - cx, dy = p.y - cy, n = Math.hypot(dx, dy);
      ctx.fillStyle = hexA(pal.dim, 0.6);
      ctx.fillRect(cx + dx / n * u - 1 * k, cy + dy / n * u - 1 * k, 2 * k, 2 * k);
      continue;
    }
    const col = returnColour(pal, p.r, stop);
    const s = (col === pal.red ? 3.4 : (col === pal.amber ? 2.8 : 2.3)) * k;
    ctx.fillStyle = col;
    ctx.fillRect(p.x - s / 2, p.y - s / 2, s, s);
  }

  // obstacle rings only - no class/distance text on the scope (operator,
  // 2026-09-27); the nearest distance is in the NEAREST corner readout
  obs.forEach((o) => {
    const [px, py] = toScreen(o.x, o.y);
    if (!onDisc(px, py)) return;
    const col = obsCol(o);
    ctx.strokeStyle = col; ctx.lineWidth = 1.5 * k;
    ctx.beginPath(); ctx.arc(px, py, 7 * k, 0, TAU); ctx.stroke();
    if (o.dynamic) {                                  // a mover gets a second ring
      ctx.setLineDash([2 * k, 2.5 * k]); ctx.lineWidth = 1 * k;
      ctx.beginPath(); ctx.arc(px, py, 10.5 * k, 0, TAU); ctx.stroke();
      ctx.setLineDash([]);
    }
  });

  // corner readouts - outside the disc, never over a return
  const pad = 7 * k;
  ctx.textBaseline = "top"; ctx.textAlign = "left";
  ctx.font = font(7, 600); ctx.fillStyle = pal.dim;
  ctx.fillText("NEAREST", pad, pad);
  ctx.font = font(14, 700);
  ctx.fillStyle = nearest === null ? pal.green : returnColour(pal, nearest, stop);
  ctx.fillText(nearest === null ? "CLEAR" : fmt(nearest, 2) + "m", pad, pad + 10 * k);

  ctx.textAlign = "right";
  ctx.font = font(7, 600); ctx.fillStyle = pal.dim;
  ctx.fillText("AVOID", W - pad, pad);
  ctx.font = font(10.5, 700); ctx.fillStyle = avoidColour(pal, av.status);
  ctx.fillText(av.status || "--", W - pad, pad + 10 * k);

  ctx.textBaseline = "bottom";
  ctx.font = font(7.5, 600); ctx.fillStyle = pal.dim;
  ctx.fillText("RANGE " + fmt(maxR, 0) + "m", W - pad, H - pad);
  if (stop) {
    ctx.fillStyle = pal.red;
    ctx.fillText("STOP " + fmt(stop, 1) + "m", W - pad, H - pad - 10 * k);
  }
  ctx.textAlign = "left";
  ctx.fillStyle = fov.enabled ? hexA(pal.green, 0.85) : pal.dim;
  ctx.fillText(fov.enabled ? "FOV " + fmt(fov.fov_deg, 0) + "°" : "FOV 360°", pad, H - pad - 10 * k);
  ctx.fillStyle = pal.dim;
  ctx.fillText(fov.enabled ? "REAR " + fmt(fov.blind_deg, 0) + "° MASKED" : "no mask", pad, H - pad);

  $("radar-title").textContent = (d.source || "SIM") + " · avoid";
  $("radar-state").textContent = av.status || "--";
  $("radar-near").textContent = nearest === null ? "clear" : ("near " + fmt(nearest, 1) + "m");
  const mult = av.status === "BRAKE" ? "0.0" : (av.status === "SLOW" ? "0.4" : "1.0");
  $("radar-mult").textContent = mult;
  $("brake-overlay").classList.toggle("show", av.status === "BRAKE");
  updateZoomLabel();
}
function updateZoomLabel() { $("rz-lvl").textContent = (12 / radarMax).toFixed(1) + "x"; }
function zoom(delta) {
  radarMax = Math.min(40, Math.max(2, radarMax + delta));
  updateZoomLabel();
  if (latest.scan !== undefined) drawRadar(latest);
}
function setupRadarControls() {
  $("rz-in").onclick = () => zoom(-2);
  $("rz-out").onclick = () => zoom(+2);
  $("rz-exp").onclick = () => {
    const on = document.body.classList.toggle("radar-expanded");
    const c = $("radar"); c.width = c.height = on ? 640 : 360;
    if (latest.scan !== undefined) drawRadar(latest);
  };
  // scroll-wheel zoom on the radar
  $("radar").addEventListener("wheel", (e) => {
    e.preventDefault();
    zoom(e.deltaY < 0 ? -1 : +1);
  }, { passive: false });
  updateZoomLabel();
}

/* ---------- safety confirm modal ---------- */
function confirmDanger(title, msg, onYes) {
  $("confirm-title").textContent = title;
  $("confirm-msg").textContent = msg;
  const ov = $("confirm-overlay"); ov.classList.add("show");
  const ok = $("confirm-ok"), cancel = $("confirm-cancel");
  const close = () => { ov.classList.remove("show"); ok.onclick = cancel.onclick = null; };
  ok.onclick = () => { close(); onYes(); };
  cancel.onclick = close;
  ov.onclick = (e) => { if (e.target === ov) close(); };
}

/* ---------- map ---------- */
let map, layers = {}, droneMarker, trailLine, wpLine, homeMarker, wpMarkers = [];
function initMap() {
  map = L.map("map", { zoomControl: false, attributionControl: false }).setView([12.9017, 77.654], 18);
  // "Dark" (CARTO dark_all) removed 2026-09-27: CARTO now answers keyless
  // requests with HTTP 200 + an "API KEY REQUIRED" placeholder tile.
  layers.street = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", { maxZoom: 19 });
  layers.sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", { maxZoom: 20 });
  layers.terrain = L.tileLayer("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", { maxZoom: 17 });
  layers.sat.addTo(map);
  trailLine = L.polyline([], { color: "#38e07b", weight: 2, opacity: 0.8 }).addTo(map);
  wpLine = L.polyline([], { color: "#22d3ee", weight: 1.5, dashArray: "5,5" }).addTo(map);
  // The delivery route is drawn separately from the editable mission line so a
  // Firebase order is visually distinct from waypoints someone dropped by hand.
  dlvRouteLine = L.polyline([], { color: "#22d3ee", weight: 2.5, opacity: 0.9 }).addTo(map);
  const arrow = L.divIcon({ className: "drone-icon", html: '<span class="drone-rot">▲</span>', iconSize: [20, 20] });
  droneMarker = L.marker([12.9017, 77.654], { icon: arrow }).addTo(map);

  document.querySelectorAll(".lyr").forEach((b) => b.onclick = () => {
    Object.values(layers).forEach((l) => map.removeLayer(l));
    layers[b.dataset.layer].addTo(map);
    document.querySelectorAll(".lyr").forEach((x) => x.classList.remove("on"));
    b.classList.add("on");
  });
  map.on("mousemove", (e) => {
    cursorLatLng = e.latlng;
    $("coord-cursor").textContent = `${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`;
  });
  map.on("click", (e) => { if (wpMode) addLocalWp(e.latlng.lat, e.latlng.lng); });
  $("tb-wp").onclick = () => { wpMode = !wpMode; $("tb-wp").classList.toggle("on", wpMode); };
  $("tb-clear").onclick = () => { localWps = []; renderWps(); send("clear_mission"); };
  $("tb-upload").onclick = uploadMission;
  $("tb-download").onclick = downloadPlan;
  $("ms-save").onclick = downloadPlan;
  $("ms-load").onclick = () => $("plan-file").click();
}
function addLocalWp(lat, lon) { localWps.push({ lat, lon, alt: 3 }); renderWps(); uploadMission(); }
function renderWps() {
  wpMarkers.forEach((m) => map.removeLayer(m));
  wpMarkers = [];
  localWps.forEach((w, i) => {
    const icon = L.divIcon({ className: "", html: `<div class="wp-icon"></div>`, iconSize: [12, 12] });
    const m = L.marker([w.lat, w.lon], { icon, draggable: true }).addTo(map);
    m.bindTooltip("WP" + (i + 1), { permanent: false });
    m.on("dragend", (e) => { const p = e.target.getLatLng(); localWps[i].lat = p.lat; localWps[i].lon = p.lng; renderWps(); uploadMission(); });
    wpMarkers.push(m);
  });
  wpLine.setLatLngs(localWps.map((w) => [w.lat, w.lon]));
}
function uploadMission() { if (localWps.length) send("upload_mission", { waypoints: localWps }); }
function updateMap(d) {
  const p = d.position || {};
  const fix = (d.telemetry || {}).gps_fix;
  const gpsValid = (fix === "2D" || fix === "3D" || fix === "RTK") && p.lat && p.lon;
  if (gpsValid) {
    droneMarker.setLatLng([p.lat, p.lon]);
    // rotate only the inner glyph (never the Leaflet-positioned outer icon)
    const rot = droneMarker._icon && droneMarker._icon.querySelector(".drone-rot");
    if (rot) rot.style.transform = `rotate(${p.heading || 0}deg)`;
    $("coord-drone").textContent = `${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}  ${fmt(p.alt, 1)}m`;
    if (!map._centered) { map.setView([p.lat, p.lon], 18); map._centered = true; }
  } else {
    $("coord-drone").textContent = "NO GPS FIX";
  }
  if (gpsValid && p.home_lat && !homeMarker) {
    homeMarker = L.marker([p.home_lat, p.home_lon],
      { icon: L.divIcon({ className: "", html: "⌂", iconSize: [16, 16] }) }).addTo(map);
  }
  // clear a stale home marker if the source/home was reset
  if (!p.home_lat && homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }
  // Up to 800 points; only re-project when the hub actually sent a new one.
  if (trailDirty) {
    trailDirty = false;
    trailLine.setLatLngs((d.trail || []).map((t) => [t[0], t[1]]));
  }
  if (!seeded && d.mission && d.mission.waypoints && d.mission.waypoints.length) {
    localWps = d.mission.waypoints.filter((w) => w.lat && w.lon).map((w) => ({ lat: w.lat, lon: w.lon, alt: w.alt }));
    renderWps(); seeded = true;
  }
}
/* ---------- delivery (Firebase order -> flight) ----------------------------
   Everything here is a read-out of the delivery node's published state: the
   browser holds no delivery state of its own, so a reload or a second operator
   opening the dashboard sees exactly the same thing. The only outbound calls
   are the four operator decisions (accept / decline / abort / auto-accept).   */
/* NOTE: LANDED means the AIRCRAFT is down, and nothing more. It used to read
   "DELIVERED", which was a claim this phase cannot support - the phase tracks
   the airframe, and a flight that never got a valid recipient handshake lands
   exactly the same way as one that did. The parcel's fate lives in
   v.outcome / v.outcome_label, rendered by updateDeliveryOutcome below. */
const DLV_LABEL = {
  IDLE: "IDLE", PENDING: "ORDER PENDING", REJECTED: "REJECTED",
  ACCEPTED: "ACCEPTED", ENROUTE: "EN ROUTE", HOVERING: "HOVERING",
  RETURNING: "RETURNING", LANDED: "LANDED", ABORTED: "ABORTED",
};

/* The two halves of the verdict, each as [text, css-class]. */
const OC_PARCEL = {
  PENDING:    ["in progress", "dim"],
  DELIVERED:  ["DELIVERED", "good"],
  FAILED:     ["NOT DELIVERED", "bad"],
  ABORTED:    ["ABORTED", "bad"],
  NEVER_FLEW: ["NEVER LAUNCHED", "warn"],
};
const OC_RETURN = {
  PENDING:      ["still flying", "dim"],
  RETURNED:     ["RETURNED HOME", "good"],
  NOT_RETURNED: ["DID NOT RETURN", "bad"],
  ON_PAD:       ["never left the pad", "warn"],
  UNKNOWN:      ["UNCONFIRMED", "warn"],
};
const DLV_TOAST = {
  PENDING: ["NEW DELIVERY ORDER", "amber"],
  ACCEPTED: ["ORDER ACCEPTED — LAUNCHING", "ok"],
  ENROUTE: ["EN ROUTE TO DROP POINT", "ok"],
  HOVERING: ["HOLDING OVER DROP POINT", "ok"],
  RETURNING: ["RETURNING HOME", "amber"],
  LANDED: ["AIRCRAFT HOME", "ok"],
  REJECTED: ["ORDER REJECTED", "bad"],
  ABORTED: ["DELIVERY ABORTED", "bad"],
};

/* The flight, as six steps. The timeline is the fastest way to answer "where
   is my order right now" without reading any numbers. */
const DLV_STEPS = ["ORDER", "LAUNCH", "EN ROUTE", "HOVER", "RETURN", "HOME"];
const DLV_STEP_OF = {
  IDLE: -1, PENDING: 0, REJECTED: 0, ACCEPTED: 1,
  ENROUTE: 2, HOVERING: 3, RETURNING: 4, LANDED: 5, ABORTED: -2,
};

function dlvAge(iso) {
  if (!iso) return "";
  const t = Date.parse(iso);
  if (isNaN(t)) return "";
  const s = Math.max(0, (Date.now() - t) / 1000);
  if (s < 60) return Math.round(s) + "s ago";
  if (s < 3600) return Math.round(s / 60) + "m ago";
  if (s < 86400) return Math.round(s / 3600) + "h ago";
  return Math.round(s / 86400) + "d ago";
}

function dlvEta(v, phase, groundSpeed) {
  // Hover time is known exactly; flight time is only ever an estimate, so it
  // is shown as one rather than dressed up with false precision.
  if (phase === "HOVERING") return fmt(v.hover_remaining_s, 0) + " s hold";
  if (!["ENROUTE", "RETURNING", "ACCEPTED"].includes(phase)) return "--";
  const remaining = v.remaining_m || 0;
  if (!remaining) return "--";
  const speed = groundSpeed > 0.6 ? groundSpeed : 4.0;   // cruise fallback
  let secs = remaining / speed;
  if (phase === "ENROUTE" || phase === "ACCEPTED") secs += (v.hover_seconds || 0);
  return "~" + (secs < 90 ? Math.round(secs) + " s" : Math.round(secs / 60) + " min");
}

function dlvTrack(phase) {
  const at = DLV_STEP_OF[phase] === undefined ? -1 : DLV_STEP_OF[phase];
  const aborted = at === -2;
  const bars = DLV_STEPS.map(function (_, i) {
    let cls = "dlv-step";
    if (aborted) cls += i === 0 ? " bad" : "";
    else if (at > i) cls += " done";
    else if (at === i) cls += " on";
    return '<div class="' + cls + '"></div>';
  }).join("");
  return '<div style="display:flex;gap:3px">' + bars + '</div>'
    + '<div class="dlv-track-labels">'
    + DLV_STEPS.map(function (l) { return "<span>" + l + "</span>"; }).join("")
    + '</div>';
}

function dlvRow(o, selected) {
  // A row is only clickable when the drone could actually fly it; a blocked
  // order still shows, with the reason, rather than being hidden.
  const cls = o.flown ? "done" : (o.dispatchable ? (selected ? "sel" : "") : "blocked");
  const who = o.recipient_id ? " · " + o.recipient_id : "";
  const coords = Number(o.target_lat).toFixed(5) + ", " + Number(o.target_lon).toFixed(5);
  const sub = o.blocked_reason
    ? o.blocked_reason
    : coords + (o.created_at ? " · " + dlvAge(o.created_at) : "");
  const pick = o.dispatchable && !o.flown ? o.order_id : "";
  return '<div class="dlv-row ' + cls + '" data-order="' + pick + '">'
    + '<span class="dlv-row-id">#' + String(o.order_id).slice(0, 14) + who + '</span>'
    + '<span class="dlv-row-dist">' + (o.distance_m ? fmt(o.distance_m, 0) + " m" : "") + '</span>'
    + '<span class="dlv-row-sub">' + sub + '</span>'
    + '</div>';
}

/* ---------- MedBox (smart payload over BLE) ---------- */
/* Everything comes from the hub's `medbox` block, which aerix-payload.service
   feeds. Advisory only: a COMPROMISED box changes this card and the console,
   never what the aircraft is allowed to do. */
const MB_LINK = { connected: "", scanning: "searching…", no_adapter: "no USB BT dongle",
                  no_key: "no command key", off: "bridge idle", error: "link error",
                  bridge_down: "bridge down", starting: "starting" };
let mbLast = { st: "--" };
function updateMedbox(d, kind) {
  const card = $("mb-card");
  if (!card) return;
  card.hidden = kind !== "medbox";
  if (card.hidden) return;
  const m = (d && d.medbox) || {}, s = m.status || null;
  const live = m.link === "connected" && !!s;
  const st = s ? s.state : "--";
  const badge = $("mb-state");
  badge.textContent = live ? st : "NO LINK";
  badge.className = "dlv-badge" + (live ? " mb-" + st.toLowerCase() : "");
  card.classList.toggle("bad", live && st === "COMPROMISED");
  card.classList.toggle("stale", !live);
  $("mb-link").textContent = live
    ? `${m.name || "MedBox"} · ${m.rssi != null ? m.rssi + " dBm" : m.adapter}`
    : (MB_LINK[m.link] || m.link || "bridge down");

  let msg = live ? "" : (m.note || "no telemetry from the box");
  if (live) {
    if (st === "COMPROMISED") msg = `UNSAFE: ${s.reason}` + (s.reason_value ? ` (${s.reason_value})` : "");
    else if (st === "DELIVERED") msg = `Delivered intact · landing ${s.landing_peak_g} g` +
                                       (s.opened_at_dest ? " · opened by recipient" : "");
    else if (st === "ARMED") msg = "Sealed and monitoring";
    else if (s.lid_closed && (s.dht_fault || s.imu_fault))   // firmware: sensorsHealthy() gates arm()
      msg = "Lid closed - WON'T ARM: " + (s.dht_fault ? "temp/humidity sensor (DHT11) not reading"
                                                      : "motion sensor (MPU6050) not answering");
    else msg = s.lid_closed ? "Lid closed - arming…" : "Unsealed - close the lid to arm";
    if (!m.key_ok) msg += " · no command key: drop window unavailable";
  }
  $("mb-msg").textContent = msg;

  // Reset: ground only (the hub refuses in the air too) and one at a time.
  const rs = m.reset || {}, armed = !!(d.telemetry && d.telemetry.armed);
  const btn = $("mb-reset");
  btn.disabled = !live || !m.key_ok || armed || rs.state === "pending";
  btn.textContent = rs.state === "pending" ? "Resetting…" : "Reset box";
  btn.title = armed ? "Locked while armed" : !live ? "Box not connected"
            : !m.key_ok ? "Bridge has no command key"
            : "Clear the box's latched state. Inspect the contents first.";
  mbLast = { st, reason: s && s.reason, value: s && s.reason_value };

  const set = (id, text, cls) => { const el = $(id); el.textContent = text; el.className = cls || ""; };
  if (!s) { ["mb-env", "mb-lid", "mb-motion", "mb-drop", "mb-batt", "mb-armed"].forEach((id) => set(id, "--")); return; }
  set("mb-env", `${s.temp_c != null ? s.temp_c.toFixed(1) + " °C" : "--"} / ${s.humidity != null ? s.humidity + " %" : "--"}`,
      s.dht_fault ? "bad" : "");
  set("mb-lid", s.lid_closed ? "closed" : "OPEN", !s.lid_closed && st === "ARMED" ? "bad" : "");
  set("mb-motion", `${(+s.shake_rms_g).toFixed(2)} g rms · max ${(+s.max_shock_g).toFixed(1)} g`,
      s.imu_fault ? "bad" : "");
  set("mb-drop", s.drop_phase === "none" ? "closed" : s.drop_phase, s.drop_phase !== "none" ? "warn" : "");
  set("mb-batt", s.batt_v != null ? `${s.batt_v.toFixed(2)} V ${s.batt_level}` : "n/a",
      s.batt_level === "CRITICAL" ? "bad" : s.batt_level === "LOW" ? "warn" : "");
  const t = s.armed_s;
  set("mb-armed", st === "ARMED" ? `${Math.floor(t / 60)}m ${t % 60}s` +
      (s.power_interruptions ? ` · ${s.power_interruptions} power cut(s)` : "") : "--",
      s.power_interruptions ? "warn" : "");
}

function updateDelivery(d) {
  const v = d.delivery || {};
  const phase = v.phase || "IDLE";
  const lower = phase.toLowerCase();
  const pending = phase === "PENDING";
  const inFlight = ["ACCEPTED", "ENROUTE", "HOVERING", "RETURNING"].includes(phase);
  const orders = v.orders || [], recent = v.recent || [];

  const badge = $("dlv-phase");
  if (badge) { badge.textContent = DLV_LABEL[phase] || phase; badge.className = "dlv-badge " + lower; }
  const hd = $("dlv-hd-phase");
  if (hd) hd.textContent = DLV_LABEL[phase] || phase;
  const link = $("dlv-link");
  if (link) { link.className = "link-dot " + (v.link || "disabled"); link.title = "order source: " + (v.link || "disabled"); }

  const count = $("dlv-count");
  if (count) {
    count.textContent = orders.length;
    count.classList.toggle("has", orders.length > 0);
    count.title = orders.length + " order(s) awaiting dispatch";
  }

  // Setup banner: the one thing that stops orders reaching the drone at all.
  const setup = $("dlv-setup");
  if (setup) {
    const broken = ["no-credentials", "error"].includes(v.link);
    setup.classList.toggle("on", broken);
    if (broken) {
      setHtml(setup, v.link === "no-credentials"
        ? "Orders from the app cannot be read yet.<br>Install the Firebase key: "
          + "<code>scripts/firebase_setup.py &lt;key.json&gt;</code>"
        : "Order source error: " + (v.last_error || "unknown"));
    }
  }

  // Inbox: every order the app has placed that the drone could still fly.
  const inbox = $("dlv-inbox");
  if (inbox) {
    const rows = orders.map(function (o) {
      return dlvRow(o, o.order_id === v.selected_order_id);
    });
    setHtml(inbox, rows.length
      ? rows.join("")
      : (v.link === "online"
          ? '<div class="dlv-empty">No orders waiting. Place one in the app.</div>'
          : ""));
  }

  $("dlv-order").textContent = v.order_id ? "#" + v.order_id : "no order";
  $("dlv-msg").textContent = v.message || "--";
  setHtml($("dlv-track"), dlvTrack(phase));
  $("dlv-target").textContent = (v.target_lat || v.target_lon)
    ? Number(v.target_lat).toFixed(5) + ", " + Number(v.target_lon).toFixed(5) : "--";
  $("dlv-dist").textContent = v.distance_m ? fmt(v.remaining_m || v.distance_m, 0) + " m" : "--";
  $("dlv-wps").textContent = v.waypoints
    ? v.waypoints + (v.fc_mission_uploaded ? " ✓FC" : "") : "--";
  $("dlv-hover").textContent = phase === "HOVERING"
    ? fmt(v.hover_remaining_s, 0) + " s left"
    : (v.hover_seconds ? fmt(v.hover_seconds, 0) + " s" : "--");
  $("dlv-eta").textContent = dlvEta(v, phase, (d.telemetry || {}).ground_speed || 0);

  // Progress bar: the hover countdown while holding, distance covered while flying.
  const wrap = $("dlv-bar-wrap"), bar = $("dlv-bar");
  if (wrap && bar) {
    let pct = null;
    if (phase === "HOVERING" && v.hover_seconds > 0) {
      pct = 100 * (1 - (v.hover_remaining_s / v.hover_seconds));
    } else if (["ENROUTE", "RETURNING"].includes(phase) && v.distance_m > 0) {
      pct = 100 * (1 - Math.min(1, (v.remaining_m || 0) / v.distance_m));
    }
    wrap.classList.toggle("on", pct !== null);
    if (pct !== null) bar.style.width = Math.max(0, Math.min(100, pct)).toFixed(1) + "%";
  }

  $("dlv-accept").disabled = !pending;
  $("dlv-reject").disabled = !pending;
  $("dlv-abort").disabled = !inFlight;
  $("dlv-auto").classList.toggle("on", !!v.auto_accept);

  updateDeliveryOutcome(v);
  updateOrderBook(v, orders, recent, phase);
  updateDeliveryMap(d, v, phase);

  // The verdict gets its own toast, because it is the thing the operator
  // actually needs told. A phase toast can only ever say the aircraft landed.
  if (lastDlvOutcome === null) {
    lastDlvOutcome = v.outcome_label || "";
  } else if ((v.outcome_label || "") !== lastDlvOutcome) {
    lastDlvOutcome = v.outcome_label || "";
    if (lastDlvOutcome) {
      const good = v.outcome === "DELIVERED" && v.return_outcome === "RETURNED";
      const bad = v.outcome === "FAILED" || v.outcome === "ABORTED"
        || v.return_outcome === "NOT_RETURNED";
      showToast(lastDlvOutcome, good ? "ok" : (bad ? "bad" : "amber"));
    }
  }

  if (lastDlvPhase === null) { lastDlvPhase = phase; return; }   // don't toast on load
  if (phase !== lastDlvPhase) {
    lastDlvPhase = phase;
    const t = DLV_TOAST[phase];
    if (t) showToast(t[0], t[1]);
  }
}

/* The verdict block. Shown from the moment a delivery is accepted so the
   handshake state is visible DURING the flight, not only after it - an
   operator who can see the drop was never authorised can abort early rather
   than watch the aircraft hover out a full countdown over the wrong garden. */
function updateDeliveryOutcome(v) {
  const box = $("dlv-outcome");
  if (!box) return;
  const parcel = v.outcome || "PENDING";
  const ret = v.return_outcome || "PENDING";
  const finished = parcel !== "PENDING";
  // Hidden only when there is genuinely nothing to say: no order in hand.
  box.classList.toggle("on", !!v.order_id);
  box.classList.toggle("final", finished);

  const hd = $("dlv-outcome-hd");
  if (hd) {
    hd.textContent = v.outcome_label || (v.order_id ? "IN PROGRESS" : "--");
    hd.className = "dlv-outcome-hd " + (OC_PARCEL[parcel] || ["", "dim"])[1];
  }
  const setCell = (id, map, key) => {
    const el = $(id);
    if (!el) return;
    const [text, cls] = map[key] || [key, "dim"];
    el.textContent = text;
    el.className = cls;
  };
  setCell("dlv-oc-parcel", OC_PARCEL, parcel);
  setCell("dlv-oc-return", OC_RETURN, ret);

  // The handshake is the ONLY evidence a parcel changed hands - the payload
  // servo is not instrumented - so it is shown raw rather than only folded
  // into the headline.
  const ble = $("dlv-oc-ble");
  if (ble) {
    ble.textContent = v.ble_verified
      ? "✓ verified"
      : (finished ? "✗ never completed" : "waiting");
    ble.className = v.ble_verified ? "good" : (finished ? "bad" : "dim");
  }

  const home = $("dlv-oc-home");
  if (home) {
    const radius = v.home_radius_m || 0;
    if (!finished || !v.ever_armed) {
      home.textContent = "--";
      home.className = "dim";
    } else {
      const dist = v.home_distance_m || 0;
      home.textContent = fmt(dist, 0) + " m" + (radius ? " / " + fmt(radius, 0) + " m" : "");
      home.className = ret === "RETURNED" ? "good" : "bad";
    }
  }

  const why = $("dlv-outcome-why");
  if (why) why.textContent = finished ? (v.outcome_reason || "") : "";
}

/* The bottom panel is the audit view: every order the app has written, queued
   and historical, so "did my order arrive" is answerable at a glance. */
function updateOrderBook(v, orders, recent, phase) {
  const book = $("dlv-book");
  if (!book) return;
  setHtml($("tbl-delivery"), "");
  const all = orders.concat(recent);
  if (!all.length) {
    setHtml(book, '<div class="dlv-empty">'
      + (v.link === "online"
          ? "No orders in Firebase yet."
          : "Order source offline — " + (v.last_error || v.link))
      + '</div>');
    return;
  }
  const rows = all.map(function (o) {
    const live = o.order_id === v.order_id && phase !== "IDLE";
    const status = live ? "flying" : String(o.status || "").toLowerCase();
    const label = live ? (DLV_LABEL[phase] || phase) : (o.status || "--");
    const pick = o.dispatchable && !o.flown;
    // Repeat: re-fly the exact same drop point. Only offered once an order
    // is done with (not live, not already sitting in the dispatch queue) -
    // it re-injects a fresh order_id at the same lat/lon, which still has to
    // go through ACCEPT & FLY like any other order, so this cannot launch
    // anything by itself.
    const canRepeat = !live && !pick && o.target_lat && o.target_lon;
    const repeatBtn = canRepeat
      ? '<button class="repeat-btn" data-repeat="1" data-repeat-id="' + o.order_id
        + '" data-repeat-lat="' + o.target_lat + '" data-repeat-lon="' + o.target_lon
        + '" title="Fly this same drop point again">↻ Repeat</button>'
      : "";
    return '<tr class="' + (live ? "live " : "") + (pick ? "pick" : "") + '"'
      + ' data-order="' + (pick ? o.order_id : "") + '">'
      + "<td>#" + String(o.order_id).slice(0, 12) + "</td>"
      + "<td>" + (o.recipient_id || "--") + "</td>"
      + "<td>" + Number(o.target_lat).toFixed(5) + ", " + Number(o.target_lon).toFixed(5) + "</td>"
      + "<td>" + (o.distance_m ? fmt(o.distance_m, 0) + " m" : "--") + "</td>"
      + "<td>" + (dlvAge(o.created_at) || "--") + "</td>"
      + '<td><span class="pill ' + status + '">' + label + "</span></td>"
      + "<td>" + repeatBtn + "</td>"
      + "</tr>";
  }).join("");
  setHtml(book, "<table><thead><tr>"
    + "<th>Order</th><th>Recipient</th><th>Target</th><th>Dist</th><th>Placed</th><th>Status</th><th></th>"
    + "</tr></thead><tbody>" + rows + "</tbody></table>");
}

function updateDeliveryMap(d, v, phase) {
  const hasTarget = (v.target_lat || v.target_lon)
    && ["PENDING", "ACCEPTED", "ENROUTE", "HOVERING", "RETURNING"].includes(phase);
  if (!hasTarget) {
    if (dlvTargetMarker) { map.removeLayer(dlvTargetMarker); dlvTargetMarker = null; }
    if (dlvRouteLine) dlvRouteLine.setLatLngs([]);
    return;
  }
  const ll = [v.target_lat, v.target_lon];
  const hovering = phase === "HOVERING";
  const mkIcon = () => L.divIcon({
    className: "",
    html: `<div class="dlv-target-icon ${hovering ? "hovering" : ""}">◎</div>`,
    iconSize: [22, 22], iconAnchor: [11, 11],
  });
  // setIcon replaces the marker's DOM node and bindTooltip rebuilds the
  // tooltip; both ran every frame for the whole flight. Only on change now.
  if (!dlvTargetMarker) {
    dlvTargetMarker = L.marker(ll, { icon: mkIcon() }).addTo(map);
    dlvTargetMarker._hov = hovering;
  } else {
    dlvTargetMarker.setLatLng(ll);
    if (dlvTargetMarker._hov !== hovering) { dlvTargetMarker.setIcon(mkIcon()); dlvTargetMarker._hov = hovering; }
  }
  if (dlvTargetMarker._tipFor !== v.order_id) {
    dlvTargetMarker._tipFor = v.order_id;
    dlvTargetMarker.bindTooltip(
      `DROP POINT${v.order_id ? " · #" + v.order_id : ""}`, { permanent: false });
  }

  // Draw from wherever the aircraft actually is, so the line shrinks as it flies.
  const p = d.position || {};
  if (p.lat && p.lon) dlvRouteLine.setLatLngs([[p.lat, p.lon], ll]);
  else dlvRouteLine.setLatLngs([ll]);
}

function initDelivery() {
  $("dlv-accept").onclick = () => {
    const v = (latest.delivery || {});
    confirmDanger("ACCEPT DELIVERY & FLY?",
      `The drone will arm, take off, fly ${fmt(v.distance_m, 0)} m to the drop point, `
      + `hold for ${fmt(v.hover_seconds, 0)} s and return home. Ensure the area is clear.`,
      () => send("delivery_accept", { order_id: v.order_id || "" }));
  };
  $("dlv-reject").onclick = () => send("delivery_reject", { reason: "declined by operator" });
  $("dlv-abort").onclick = () => confirmDanger("ABORT DELIVERY?",
    "The drone will stop the delivery and return home immediately.",
    () => send("delivery_abort"));
  $("dlv-auto").onclick = () => {
    const on = !!(latest.delivery || {}).auto_accept;
    if (on) { send("delivery_set_auto", { enabled: false }); return; }
    confirmDanger("ENABLE AUTO-ACCEPT?",
      "Every new order will be flown WITHOUT asking. The drone can arm and take "
      + "off on its own from this point on.",
      () => send("delivery_set_auto", { enabled: true }));
  };
  // Test order: drops an order at the map cursor without touching Firebase, so
  // the whole chain can be demonstrated with no app and no credentials.
  $("dlv-inject").onclick = () => {
    if (!cursorLatLng) { showToast("MOVE THE CURSOR OVER THE MAP FIRST", "amber"); return; }
    const { lat, lng } = cursorLatLng;
    send("delivery_inject", { lat, lon: lng, order_id: "test-" + Date.now() });
  };

  // Ask Firebase now rather than waiting out the poll interval - the first
  // thing anyone does after placing an order in the app is look here.
  const refresh = $("dlv-refresh");
  if (refresh) refresh.onclick = () => {
    refresh.classList.add("spin");
    setTimeout(() => refresh.classList.remove("spin"), 600);
    send("delivery_refresh");
  };

  // Picking an order out of the queue. Delegated, because the rows are
  // re-rendered on every frame and per-row handlers would not survive.
  // Selecting only moves the offer; flying still needs ACCEPT & FLY.
  const pick = (ev) => {
    const repeatBtn = ev.target.closest("[data-repeat]");
    if (repeatBtn) {
      const lat = parseFloat(repeatBtn.getAttribute("data-repeat-lat"));
      const lon = parseFloat(repeatBtn.getAttribute("data-repeat-lon"));
      if (isNaN(lat) || isNaN(lon)) return;
      send("delivery_inject", {
        lat, lon,
        order_id: "repeat-" + repeatBtn.getAttribute("data-repeat-id") + "-" + Date.now(),
      });
      showToast("REPEAT ORDER QUEUED · same drop point · ACCEPT & FLY to launch", "good");
      return;
    }
    const row = ev.target.closest("[data-order]");
    if (!row) return;
    const id = row.getAttribute("data-order");
    if (!id) return;
    send("delivery_select", { order_id: id });
  };
  const inbox = $("dlv-inbox"); if (inbox) inbox.onclick = pick;
  const book = $("dlv-book");   if (book) book.onclick = pick;
}

function downloadPlan() {
  fetch("/api/mission/plan").then((r) => r.json()).then((plan) => {
    const blob = new Blob([JSON.stringify(plan, null, 2)], { type: "application/json" });
    const a = document.createElement("a"); a.href = URL.createObjectURL(blob);
    a.download = "mission.plan"; a.click();
  });
}
$("plan-file") && ($("plan-file").onchange = (e) => {
  const f = e.target.files[0]; if (!f) return;
  const rd = new FileReader();
  rd.onload = () => {
    try {
      const plan = JSON.parse(rd.result);
      fetch("/api/mission/plan", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(plan) })
        .then((r) => r.json()).then(() => { seeded = false; });
    } catch (err) { nlLog("✗ invalid .plan file", false); }
  };
  rd.readAsText(f);
});

/* ---------- panels ---------- */
/* A readout is a magnitude and a unit, and they do not deserve the same
   weight. Split a trailing unit off so CSS can set it small and dim - the
   number is what the operator scans for. Values that are already markup
   (pills, spans) are passed through untouched. */
const KV_UNIT = /^(-{2}|[-+]?\d[\d.,]*)\s*(m\/s|°C|µs|ch|%|m|s|°|V)$/;
function kvVal(v) {
  if (v === null || v === undefined || v === "") return "--";
  const s = String(v);
  if (s.indexOf("<") >= 0) return s;
  const m = KV_UNIT.exec(s.trim());
  return m ? m[1] + '<span class="u">' + m[2] + "</span>" : s;
}
function esc(v) {
  return String(v).replace(/[&<>"']/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);
}
function kv(rows) {
  return rows.map(([k, v]) => `<tr><td>${k}</td><td>${kvVal(v)}</td></tr>`).join("");
}
function updatePanels(d) {
  const t = d.telemetry || {}, av = d.avoidance || {}, m = d.mission || {}, h = d.health || {};
  // Transmitter link. The navigator stands down the moment the pilot moves the
  // mode switch, so whether that switch can reach the aircraft is worth seeing
  // on the ground, not discovering in the air.
  const rc = d.rc || {};
  const rcTxt = rc.connected
    ? `<span class="pill good">LINKED</span> ${rc.count || 0} ch`
    : '<span class="pill bad">NO RC</span>';
  setHtml($("tbl-telem"), kv([
    ["Flight Mode", t.flight_mode], ["Armed", t.armed ? "YES" : "no"],
    ["Transmitter", rcTxt],
    ["GPS Fix", t.gps_fix], ["Satellites", t.satellites],
    ["Altitude", fmt(t.altitude, 1) + " m"], ["Ground Spd", fmt(t.ground_speed, 1) + " m/s"],
    ["Vert Spd", fmt(t.vert_speed, 1) + " m/s"], ["Heading", fmt(t.heading, 0) + "°"],
    ["Pitch", fmt(t.pitch, 1) + "°"], ["Roll", fmt(t.roll, 1) + "°"],
  ]));
  setHtml($("tbl-avoid"), kv([
    ["Avoidance", av.enabled ? "ON" : "OFF"], ["Status", av.status],
    ["Sending Cmds", av.sending ? "yes" : "no"], ["Direction", av.direction],
    ["Obstacles", av.count], ["Closest", fmt(av.closest_m, 2) + " m"],
    ["TTC", fmt(av.ttc_s, 2) + " s"], ["CPA", fmt(av.cpa_m, 2) + " m"],
    ["Command", av.command], ["Reason", av.reason || "--"],
  ]));
  // Altitude hardlock + who is flying. Shown on every frame: during real
  // flight the operator needs to see the limit and the override state without
  // having to infer them from the aircraft's behaviour.
  const ceil = Number(m.alt_ceiling_m) || 0;
  const altNow = Number(t.altitude) || 0;
  const lockTxt = ceil
    ? `${fmt(altNow, 2)} m / ${fmt(ceil, 1)} m ceiling`
    : "--";
  const lockCls = ceil && altNow > ceil + 0.05 ? "bad"
                : ceil && altNow > ceil * 0.85 ? "warn" : "good";
  const rows = [
    ["Phase", m.phase], ["Waypoint", `${m.current_wp}/${m.total_wp}`],
    ["Altitude", `<span class="pill ${lockCls}">${lockTxt}</span>`],
    ["Control", m.pilot_override
      ? '<span class="pill bad">PILOT (transmitter)</span>'
      : '<span class="pill good">AUTO (ground station)</span>'],
    ["Avoiding", m.avoiding ? "yes" : "no"], ["Status", m.message || "--"],
  ];
  if (m.arm_refusal) rows.push(["Autopilot", `<span class="pill warn">${m.arm_refusal}</span>`]);
  const rf = d.rangefinder || {};
  if (rf.enabled) {
    rows.push(["TF02 AGL", rf.valid
      ? `<span class="pill good">${fmt(rf.agl_m, 2)} m · str ${rf.strength}</span>`
      : `<span class="pill ${rf.age_s != null && rf.age_s < 1 ? "warn" : "bad"}">${esc(rf.message || "no reading")}</span>`]);
  }
  if (m.drop_stage) {
    const dcls = m.drop_stage === "done" ? "good" : m.drop_stage === "aborted" ? "bad" : "warn";
    rows.push(["Drop", `<span class="pill ${dcls}">${esc(m.drop_stage.toUpperCase())}</span> ${esc(m.drop_note || "")}`]);
  }
  if (m.drop_stage !== lastDropStage) {
    if (lastDropStage !== null) {
      if (m.drop_stage === "release") showToast("PARCEL RELEASED · delivery complete", "good");
      else if (m.drop_stage === "aborted") showToast("PRECISION DROP ABORTED · " + (m.drop_note || ""), "warn");
      else if (m.drop_stage === "approach") showToast("PRECISION DROP · moving beside the recipient", "warn");
    }
    lastDropStage = m.drop_stage;
  }
  setHtml($("tbl-mission"), kv(rows));

  // The override is latched until a human clears it, so make the way out
  // impossible to miss: highlight RESUME for exactly as long as it is the
  // thing standing between the operator and a flyable aircraft.
  const resumeBtn = $("btn-resume");
  if (resumeBtn) {
    resumeBtn.classList.toggle("amber", !!m.pilot_override);
    resumeBtn.textContent = m.pilot_override
      ? "RESUME · TAKE BACK CONTROL"
      : "RESUME";
  }

  if (m.pilot_override !== lastOverride) {
    if (lastOverride !== null) {
      showToast(m.pilot_override
        ? "PILOT OVERRIDE · transmitter has control · press RESUME to fly again"
        : "AUTO · ground station has control",
        m.pilot_override ? "warn" : "good");
    }
    lastOverride = m.pilot_override;
  }
  setHtml($("tbl-health"), kv([
    ["CPU", fmt(h.cpu, 0) + " %"], ["RAM", fmt(h.ram, 0) + " %"],
    ["Temp", fmt(h.temp, 0) + " °C"], ["LIDAR FPS", fmt(h.lidar_fps, 1)],
    ["Radar FPS", fmt(h.radar_fps, 1)], ["MAVLink FPS", fmt(h.mavlink_fps, 1)],
    ["TF02 FPS", fmt(h.rangefinder_fps, 1)],
  ]));
  $("mode-actual").textContent = t.flight_mode || "--";
  $("mode-req").textContent = m.phase || "--";
  document.querySelectorAll("#mode-grid button").forEach((b) =>
    b.classList.toggle("on", b.dataset.mode === t.flight_mode));

  // announce flight-mode changes on-screen (skip the first render so it
  // doesn't fire just because the page loaded)
  const mode = t.flight_mode || null;
  if (mode && mode !== lastMode) {
    if (lastMode !== null) showToast("MODE · " + mode, "good");
    lastMode = mode;
  }
}

/* ---------- header ---------- */
function updateHeader(d) {
  const t = d.telemetry || {}, link = d.link || {};
  setPill($("pill-link"), link.connected ? "LINK OK" : "LINK LOST", link.connected ? "good" : "bad");
  // This laptop's link to the Pi, measured by the Pi from our own acks: the
  // extra round trip is queue. It is what every button press waits through.
  const net = d.net, gp = $("pill-gcs");
  if (gp && net && performance.now() - lastMsgAt < WS_STALE_MS) {
    const q = net.qdelay_ms || 0;
    setPill(gp, net.rtt_ms == null ? "GCS --" : `GCS ${net.rtt_ms}ms`,
            q > 250 ? "bad" : q > 100 ? "warn" : "good");
    gp.title = `GCS link round trip ${net.rtt_ms} ms, ${q} ms of it queueing` +
               (reconnects ? ` · ${reconnects} dead-link reconnect(s)` : "");
  }
  setPill($("pill-source"), d.source || "SIM", d.sim ? "warn" : "good");
  setPill($("pill-mode"), t.flight_mode || "--", "");
  setPill($("pill-armed"), t.armed ? "ARMED" : "DISARMED", t.armed ? "good" : "bad");
  window.setPropTelemetry && window.setPropTelemetry(t);
  $("chip-sat").textContent = t.satellites || 0;
  $("chip-batt").textContent = fmt(t.battery_v, 1);
  $("chip-pkt").textContent = t.packets || 0;
  $("chip-cpu").textContent = fmt(d.health ? d.health.cpu : 0, 0);
  $("chip-clock").textContent = new Date().toLocaleTimeString();
  updateHud(d);
  $("av-on").classList.toggle("on", av_enabled(d));
  $("av-off").classList.toggle("on", !av_enabled(d));
  $("src-sim").classList.toggle("on", d.sim);
  $("src-real").classList.toggle("on", !d.sim);
}
function av_enabled(d) { return d.avoidance ? d.avoidance.enabled : true; }

/* ---------- primary flight band ----------
   The six numbers an operator glances at without reading a table. Duplicated
   from the telemetry panel on purpose: that panel is a reference, this is the
   instrument. State classes only ever come from the data, never from styling. */
function hudSet(id, val, cls) {
  const el = $(id);
  if (!el) return;
  el.textContent = val;
  const box = el.closest(".hud-cell");
  if (box) box.className = "hud-cell" + (cls ? " " + cls : "");
}
function updateHud(d) {
  const t = d.telemetry || {}, m = d.mission || {}, link = d.link || {};
  const ceil = Number(m.alt_ceiling_m) || 0, alt = Number(t.altitude) || 0;
  hudSet("hud-alt", fmt(t.altitude, 1),
    ceil && alt > ceil + 0.05 ? "bad" : ceil && alt > ceil * 0.85 ? "warn" : "");
  hudSet("hud-gs", fmt(t.ground_speed, 1));
  hudSet("hud-vs", fmt(t.vert_speed, 1));
  hudSet("hud-hdg", fmt(t.heading, 0));
  const bv = Number(t.battery_v) || 0;
  hudSet("hud-batt", fmt(t.battery_v, 1), bv && bv < 21 ? "bad" : bv && bv < 22 ? "warn" : "good");
  const sats = Number(t.satellites) || 0;
  hudSet("hud-sat", sats, sats >= 8 ? "good" : sats >= 5 ? "warn" : "bad");
  // TF02-Pro AGL: green = valid return, amber = sensor talking but no target
  // (4500 cm code / weak), red = silent. "off" when rangefinder.enabled=false.
  const rf = d.rangefinder || {};
  const dropping = !!(m.drop_stage && !["done", "aborted"].includes(m.drop_stage));
  const aglCls = "hud-agl" + (dropping ? " drop" : "");
  if (!rf.enabled) hudSet("hud-agl", "off", aglCls);
  else if (rf.valid) hudSet("hud-agl", fmt(rf.agl_m, 2), aglCls + " good");
  else hudSet("hud-agl", "--", aglCls + (rf.age_s != null && rf.age_s < 1 ? " warn" : " bad"));
  const ring = $("hud-ring");
  if (ring) ring.style.transform = "rotate(" + (Number(t.heading) || 0) + "deg)";
  const st = $("hud-state");
  if (st) {
    st.textContent = t.armed ? "ARMED" : link.connected ? "STANDBY" : "NO LINK";
    st.className = "hud-state " + (t.armed ? "armed" : link.connected ? "ready" : "down");
  }
}

/* ---------- sparklines ---------- */
const sparkDefs = [["CPU %", "cpu"], ["RAM %", "ram"], ["Batt V", "battery_v"], ["LIDAR Hz", "lidar_fps"], ["MAVLink Hz", "mavlink_fps"], ["Net ms", "net"]];
const sparkData = {}, sparkCanvas = {};
// Resolved once per theme, not per sample: getComputedStyle forces a style
// recalculation and this ran six times a frame. Cleared by the theme toggle.
let sparkColor = null;
function buildSparks() {
  const wrap = $("health-spark");
  sparkDefs.forEach(([label, key]) => {
    sparkData[key] = [];
    const row = document.createElement("div"); row.className = "spark";
    row.innerHTML = `<span class="lbl">${label}</span><canvas width="120" height="18"></canvas><span class="val" id="sv-${key}">--</span>`;
    wrap.appendChild(row);
    sparkCanvas[key] = row.querySelector("canvas");
  });
}
function pushSpark(key, val, max) {
  const arr = sparkData[key]; if (!arr) return;
  arr.push(val); if (arr.length > 60) arr.shift();
  $("sv-" + key).textContent = fmt(val, key === "battery_v" ? 1 : 0);
  const cv = sparkCanvas[key];
  const ctx = cv.getContext("2d"); ctx.clearRect(0, 0, cv.width, cv.height);
  // read the accent off the stylesheet so the light theme is not drawn in a
  // colour picked for the dark one
  if (!sparkColor) sparkColor = (getComputedStyle(document.body).getPropertyValue("--cyan") || "#22d3ee").trim();
  ctx.strokeStyle = sparkColor;
  ctx.lineWidth = 1.25; ctx.beginPath();
  const m = max || Math.max(1, ...arr);
  arr.forEach((v, i) => { const x = i / 59 * cv.width, y = cv.height - (v / m) * (cv.height - 2) - 1; i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
}
function updateSparks(d) {
  const h = d.health || {}, t = d.telemetry || {};
  pushSpark("cpu", h.cpu || 0, 100); pushSpark("ram", h.ram || 0, 100);
  pushSpark("battery_v", t.battery_v || 0, 17); pushSpark("lidar_fps", h.lidar_fps || 0);
  pushSpark("mavlink_fps", h.mavlink_fps || 0); pushSpark("net", netLatency, 200);
}

/* ---------- console ---------- */
function updateConsole(d) {
  const box = $("console");
  let added = 0;
  (d.console || []).forEach((e) => {
    if (e.id <= lastConsoleId) return;
    lastConsoleId = e.id;
    added++;
    const div = document.createElement("div");
    const ts = new Date(e.t * 1000).toLocaleTimeString();
    div.innerHTML = `<span class="t">${ts}</span> <span class="s">${e.src}</span> <span class="${e.level}">${e.msg}</span>`;
    box.appendChild(div);

    // pop aux-switch / mode events from the flight controller on-screen too
    // (E-Stop, AutoTune attempts, mode-change rejects) so transmitter actions
    // are easy to track even when they aren't a flight-mode change. Skip the
    // first pass so the console backlog doesn't flood the screen on load.
    if (seededConsole && e.src === "mavlink.link") {
      const m = (e.msg || "").replace(/^FC:\s*/, "");
      if (/^RC\d+:/.test(m) || /Mode change to .+ (failed|denied)/i.test(m)) {
        const cls = e.level === "error" ? "bad" : (e.level === "warning" ? "warn" : "good");
        showToast(m, cls);
      }
    }
  });
  // Reading scrollHeight forces a synchronous layout of the page; it ran
  // every frame whether or not a line arrived.
  if (added) {
    while (box.children.length > 200) box.removeChild(box.firstChild);
    box.scrollTop = box.scrollHeight;
  }
  seededConsole = true;
}

/* ---------- cameras (live MJPEG + detection overlay) ---------- */
function initCams() {
  // Pi camera only. The USB webcam (cam 0) was removed 2026-09-11; the Pi cam
  // keeps id 1, so its stream URL is unchanged.
  const openers = [];
  // The WebSocket watchdog calls this when the link died silently: the
  // camera's TCP connection went down the same hole and would otherwise sit
  // on its last frame just as long.
  reopenCams = () => { if (!document.hidden) openers.forEach((f) => f()); };
  [1].forEach((i) => {
    const img = $("camimg" + i);
    const open = () => { img.src = "/api/camera/" + i + "/stream?t=" + Date.now(); };
    openers.push(open);
    img.src = "/api/camera/" + i + "/stream";
    img.onerror = () => { setTimeout(() => { if (!document.hidden) open(); }, 2000); };
    // A background tab keeps downloading MJPEG at full rate. Measured 09-23:
    // three streams to one Mac (12.6 Mbit/s) sharing the Wi-Fi, and the
    // adaptive controller steers by the WORST viewer, so a forgotten tab
    // softened and slowed the picture in the tab actually being watched.
    // Hidden -> drop the stream; visible -> a fresh one (newest frame, no backlog).
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) img.removeAttribute("src");
      else open();
    });
  });
}
function updateCam(i, cam) {
  const lbl = $("camlbl" + i), ov = $("camov" + i);
  if (!lbl || !ov) return;
  const prefix = "P1";
  if (cam && cam.connected) {
    lbl.textContent = `${prefix} ${cam.res} ${cam.fps}fps` + (cam.stab ? " STAB" : "");
  } else {
    lbl.textContent = `${prefix} offline`;
  }
  // NPU chip: what the Hailo person lock is doing. The box itself is burned
  // into the video (and the replay), so it stays locked to the picture.
  const gpu = lbl.parentElement && lbl.parentElement.querySelector(".cam-gpu");
  if (gpu) {
    const v = cam && cam.vision;
    if (!v) gpu.textContent = "NPU OFF";
    else if (v.npu !== "ready") gpu.textContent = "NPU " + String(v.npu || "--").toUpperCase();
    else if (v.lock === "lock") gpu.textContent = `LOCK ${Math.round((v.score || 0) * 100)}% · ${fmt(v.det_hz, 0)}Hz`;
    else if (v.lock === "hold") gpu.textContent = `HOLD · ${fmt(v.det_hz, 0)}Hz`;
    else gpu.textContent = `NPU ${fmt(v.det_hz, 0)}Hz`;
    gpu.title = v ? `Hailo-8 person lock: ${v.infer_ms} ms/inference` : "";
  }
  const dets = (cam && cam.detections) || [];
  const detKey = JSON.stringify(dets);
  if (ov._dets === detKey) return;
  ov._dets = detKey;
  ov.innerHTML = "";
  dets.forEach((b) => {
    const box = document.createElement("div");
    box.className = "cam-box";
    box.style.left = (b.x * 100) + "%"; box.style.top = (b.y * 100) + "%";
    box.style.width = (b.w * 100) + "%"; box.style.height = (b.h * 100) + "%";
    const span = document.createElement("span");
    span.textContent = `${b.label} LOCK ${fmt(b.conf, 2)}`;
    box.appendChild(span); ov.appendChild(box);
  });
}

/* ---------- NL box ---------- */
function nlLog(text, ok) {
  const log = $("nl-log"), div = document.createElement("div");
  div.style.color = ok === undefined ? "var(--dim)" : (ok ? "var(--green)" : "var(--red)");
  div.textContent = text; log.prepend(div);
  while (log.children.length > 20) log.removeChild(log.lastChild);
}
function sendNL() {
  const el = $("nl-input"), text = el.value.trim(); if (!text) return;
  nlLog("> " + text); send("nl", { text }); el.value = "";
  nlHistoryPush(text);
}
function nlHistoryPush(text) {
  if (nlHistory[nlHistory.length - 1] !== text) {
    nlHistory.push(text);
    if (nlHistory.length > NL_HISTORY_MAX) nlHistory.shift();
    try { localStorage.setItem(NL_HISTORY_KEY, JSON.stringify(nlHistory)); } catch {}
  }
  nlHistoryIdx = nlHistory.length;
  nlDraft = "";
}
function nlHistoryNav(dir) {
  const el = $("nl-input");
  if (nlHistory.length === 0) return;
  if (nlHistoryIdx === nlHistory.length) nlDraft = el.value;  // stash in-progress text
  const next = nlHistoryIdx + dir;
  if (next < 0 || next > nlHistory.length) return;             // clamp at both ends
  nlHistoryIdx = next;
  el.value = nlHistoryIdx === nlHistory.length ? nlDraft : nlHistory[nlHistoryIdx];
  requestAnimationFrame(() => el.setSelectionRange(el.value.length, el.value.length));
}

/* ---------- save scan ---------- */
function saveScan() {
  const scan = latest.scan || null;
  const payload = { ts: Date.now() / 1000, source: latest.source, scan, obstacles: latest.obstacles || [] };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const a = document.createElement("a"); a.href = URL.createObjectURL(blob);
  a.download = "scan_" + Date.now() + ".json"; a.click();
  nlLog("✓ scan saved", true);
}

/* ---------- downward LiDAR (TF02-Pro) panel, under the radar ----------
   Same three states as the HUD cell: valid (green), talking but no target -
   the 4500 cm code / weak return - (amber), silent (red); "off" when
   rangefinder.enabled is false. Scale runs from the ground to one metre over
   the altitude ceiling, SQUARE-ROOT spaced: the 0.6-2 m band where the drop
   happens gets ~40% of the bar instead of ~20% on a linear scale. Drop marks
   come from the hub (delivery.drop_target_agl_m / drop_abort_agl_m). */
function aglPos(h, top) {
  return Math.sqrt(Math.max(0, Math.min(h, top)) / top) * 100;
}
function updateAglPanel(d) {
  const panel = $("agl-panel");
  if (!panel) return;
  const rf = d.rangefinder || {}, m = d.mission || {}, h = d.health || {};
  const top = (Number(m.alt_ceiling_m) || 5) + 1;
  const tgt = Number(rf.target_agl_m) || 0, abort = Number(rf.abort_agl_m) || 0;
  const agl = Number(rf.agl_m);
  let state, label;
  if (!rf.enabled) { state = ""; label = "OFF"; }
  else if (rf.valid) { state = "good"; label = "OK"; }
  else if (rf.age_s != null && rf.age_s < 1) { state = "warn"; label = "NO TARGET"; }
  else { state = "bad"; label = "SILENT"; }
  const low = state === "good" && abort > 0 && agl < abort;
  panel.className = "panel agl-panel" + (state ? " " + state : "") + (low ? " low" : "");
  $("agl-state").textContent = label;
  $("agl-val").textContent = !rf.enabled ? "off" : rf.valid ? fmt(agl, 2) : "--";
  $("agl-fill").style.width = (rf.valid ? aglPos(agl, top) : 0).toFixed(1) + "%";
  for (const [id, v] of [["agl-mk-target", tgt], ["agl-mk-abort", abort]]) {
    const mk = $(id);
    mk.classList.toggle("show", v > 0);
    if (v > 0) mk.style.left = aglPos(v, top).toFixed(1) + "%";
  }
  // Tick labels: ground, abort floor, drop height, ceiling. Rebuilt only when
  // the marks or ceiling change (setHtml skips identical markup).
  const ticks = [[0, "0", ""]];
  if (abort > 0) ticks.push([abort, fmt(abort, 1), "abort"]);
  if (tgt > 0) ticks.push([tgt, fmt(tgt, 1), "target"]);
  ticks.push([top - 1, fmt(top - 1, 0) + " m", ""]);
  setHtml($("agl-scale"), ticks.map(([v, t, c]) =>
    `<span class="${c}" style="left:${aglPos(v, top).toFixed(1)}%">${t}</span>`).join(""));
  const meta = [];
  if (rf.enabled) {
    if (rf.valid || rf.strength) meta.push(`str ${rf.strength}`);
    if (rf.temp_c != null) meta.push(`${fmt(rf.temp_c, 0)}°C`);
    meta.push(`${fmt(h.rangefinder_fps, 1)} Hz`);
    if (!rf.valid && rf.age_s != null && rf.age_s >= 1) meta.push(`last ${fmt(rf.age_s, 1)} s ago`);
    else if (!rf.valid && rf.message) meta.push(rf.message);
    if (m.drop_stage) meta.push(`drop ${m.drop_stage}`);
  } else meta.push("rangefinder.enabled = false");
  $("agl-meta").textContent = meta.join(" · ");
}

/* ---------- render ---------- */
function render(d) {
  if (srvAdoptCfg) srvAdoptCfg(d);
  if (aux2AdoptCfg) aux2AdoptCfg(d);
  if (altAdoptState) altAdoptState(d);
  if (recAdopt) recAdopt(d);
  updateHeader(d); drawRadar(d); updateAglPanel(d); updateMap(d); updatePanels(d); updateDelivery(d);
  updateSparks(d); updateConsole(d);
  // cameras[] is keyed by position, not by camera id, and now holds exactly
  // one entry (the Pi cam) - so index 0 feeds the tile whose DOM id is 1.
  updateCam(1, (d.cameras || [])[0]);
}

/* ---------- flight recording (auto on ARM, one clip kept) ---------- */
/* The aircraft records itself: the hub starts a recording on the ARM edge and
   finishes it on disarm, whether or not this page is open. Nothing here can
   start or stop a flight recording - this is a read-out plus a replay window.
   Every number displayed comes from the hub's `recording` block. */
function initRecording() {
  const dot = $("rec-dot"), label = $("rec-label"), btn = $("rec-toggle");
  const modal = $("rec-modal"), video = $("rec-video"), meta = $("rec-modal-meta");
  const empty = $("rec-empty"), dl = $("rec-dl"), closeBtn = $("rec-close");
  if (!btn || !modal || !video) return;

  let clipKey = "";      // identifies WHICH clip is currently loaded
  let clipSeq = null;    // which FLIGHT it is (recorder's monotonic number)
  let upgradeWaiting = false;  // HD version landed while the operator watched
  let haveClip = false;
  let lastState = null;  // for the "clip ready" toast

  const mmss = (s) => {
    s = Math.max(0, Math.round(Number(s) || 0));
    return String(Math.floor(s / 60)).padStart(2, "0") + ":" +
           String(s % 60).padStart(2, "0");
  };

  const loadClip = () => {
    if (haveClip) {
      // Keyed by the clip's own end time rather than Date.now(): the url is
      // fixed and its contents change every flight, so a stable per-clip key
      // lets the browser reuse bytes while seeking but can never serve the
      // PREVIOUS flight back. The server sends no-store for the same reason.
      video.src = "/api/recording/video?c=" + encodeURIComponent(clipKey);
      video.poster = "/api/recording/poster.jpg?c=" + encodeURIComponent(clipKey);
      video.style.display = ""; empty.style.display = "none"; dl.style.display = "";
    } else {
      video.removeAttribute("src"); video.removeAttribute("poster");
      video.style.display = "none"; empty.style.display = ""; dl.style.display = "none";
    }
  };

  const openModal = () => {
    modal.classList.add("show");
    btn.setAttribute("aria-pressed", "true");
    loadClip();
  };
  const closeModal = () => {
    modal.classList.remove("show");
    btn.setAttribute("aria-pressed", "false");
    // Tear the source down rather than just hiding it: a hidden <video> left
    // playing keeps a decoder running on the operator's laptop, which in the
    // field is the same machine holding the live stream open.
    try { video.pause(); video.removeAttribute("src"); video.load(); } catch (e) {}
  };

  // The stabilised HD render replaces the quick clip of the SAME flight.
  // Swapping mid-playback would jump the operator back to 00:00, so an
  // upgrade waits for a pause or the end; a NEW flight never waits.
  const takeUpgrade = () => {
    if (upgradeWaiting && modal.classList.contains("show")) { upgradeWaiting = false; loadClip(); }
  };
  video.addEventListener("pause", takeUpgrade);
  video.addEventListener("ended", takeUpgrade);

  btn.onclick = () => (modal.classList.contains("show") ? closeModal() : openModal());
  closeBtn.onclick = closeModal;
  modal.onclick = (e) => { if (e.target === modal) closeModal(); };
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && modal.classList.contains("show")) closeModal();
  });

  recAdopt = (d) => {
    const r = (d && d.recording) || null;
    if (!r) return;
    const clip = r.clip || null;
    haveClip = !!clip;

    // seq + stabilized: seq says WHICH flight (monotonic - the Pi has no RTC,
    // so "ended" alone could repeat or go backwards across boots), stabilized
    // says which version of it. Both change the bytes behind the fixed url.
    const key = clip ? [clip.seq || 0, clip.stabilized ? "hd" : "q", clip.ended || ""].join(":") : "";
    if (key !== clipKey) {
      const newFlight = !clip || clip.seq !== clipSeq;
      clipKey = key;
      clipSeq = clip ? clip.seq : null;
      if (clip && newFlight && lastState !== null) {
        nlLog("✓ flight clip ready — " + mmss(clip.duration_s), true);
      }
      if (modal.classList.contains("show")) {
        if (newFlight || video.paused || video.ended) { upgradeWaiting = false; loadClip(); }
        else upgradeWaiting = true;
      }
    }
    lastState = r.state;

    let text, cls;
    if (!r.enabled)                  { text = "REC OFF";  cls = "off"; }
    else if (r.state === "recording"){ text = "REC " + mmss(r.elapsed_s); cls = "live"; }
    else if (r.state === "encoding") {
      text = r.progress != null ? "RENDER " + Math.round(r.progress * 100) + "%" : "ENCODING";
      cls = "busy";
    }
    else if (r.state === "error")    { text = "REC ERR";  cls = "err"; }
    // Ahead of the CLIP branch deliberately. A flight that is recorded but not
    // yet rendered must NOT read as "CLIP 00:22" - that is precisely how the
    // dashboard spent four days offering a bench clip in place of the flight
    // the operator had just landed. Say the newer one is still owed.
    else if (r.pending && !r.pending.quick_done) { text = "REPLAY OWED"; cls = "busy"; }
    // The latest flight is playable; its stabilised HD version is on its way.
    else if (clip && r.upgrading != null) {
      text = "CLIP " + mmss(clip.duration_s) + " · HD " + Math.round(r.upgrading * 100) + "%";
      cls = "ready";
    }
    else if (clip)                   { text = "CLIP " + mmss(clip.duration_s); cls = "ready"; }
    else                             { text = "REC IDLE"; cls = "off"; }
    label.textContent = text;
    dot.className = "rec-dot " + cls;
    // The hub's own explanation of the current state - including why a short
    // clip was discarded, which is otherwise invisible.
    label.title = r.message || "";

    const flown = clip && clip.ended ? new Date(clip.ended * 1000) : null;
    meta.textContent = clip
      ? (flown ? flown.toLocaleString([], { month: "short", day: "numeric",
                  hour: "2-digit", minute: "2-digit" }) + " · " : "") +
        (clip.stabilized ? "stabilised" : (r.upgrading != null ? "quick · HD rendering" : "quick")) + " · " +
        mmss(clip.duration_s) + " · " + clip.width + "×" + clip.height + " · " +
        clip.fps + " fps · " + (clip.size_bytes / 1e6).toFixed(0) + " MB" +
        (clip.dropped ? " · " + clip.dropped + " frames dropped" : "")
      : "";
  };
}

/* ---------- logs list ---------- */
function loadLogList() {
  fetch("/api/logs").then((r) => r.json()).then((d) => {
    const sel = $("log-select"); sel.innerHTML = '<option value="">-- select log --</option>';
    (d.logs || []).forEach((f) => { const o = document.createElement("option"); o.value = f; o.textContent = f; sel.appendChild(o); });
  });
}

/* ---------- wiring ---------- */
function wire() {
  document.querySelectorAll("[data-cmd]").forEach((b) => {
    const cmd = b.dataset.cmd;
    b.addEventListener("click", () => {
      if (cmd === "arm") {
        confirmDanger("ARM MOTORS?",
          "The propellers will spin up immediately. Ensure the area is clear.",
          () => send("arm"));
        return;
      }
      send(cmd);
    });
  });
  $("nl-send").onclick = sendNL;
  $("nl-input").addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); sendNL(); }
    else if (e.key === "ArrowUp") { e.preventDefault(); nlHistoryNav(-1); }
    else if (e.key === "ArrowDown") { e.preventDefault(); nlHistoryNav(1); }
  });
  // "AUTO MODE · READY FOR NEXT MISSION": one click = clear a latched
  // transmitter takeover (resume) + a stuck delivery-in-progress flag
  // (delivery_reset, itself gated to a disarmed aircraft server-side, so
  // this can never interrupt a real flight). Replaces "restart the Pi" as
  // the way back to flyable after an override, an abort, or a finish.
  $("btn-auto-mode").onclick = () => {
    send("resume");
    send("delivery_reset");
    showToast("AUTO MODE · clearing takeover + delivery state", "good");
  };
  $("src-sim").onclick = () => send("set_source", { mode: "sim" });
  $("src-real").onclick = () => confirmDanger("SWITCH TO REAL MODE?",
    "Commands will control the PHYSICAL drone (Pixhawk + LIDAR). Simulation safety is disabled.",
    () => send("set_source", { mode: "real" }));
  // Payload servo (AUX4 = output channel 12 since 2026-09-11). Every number
  // here comes from the `payload` block the hub sends in each frame -
  // config/default.yaml is the single source of truth. These used to be
  // hardcoded constants and drifted out of step with the YAML and with
  // docs/PROJECT_NOTES.md, which by August disagreed three different ways.
  //
  // srvCfg is a fallback only, for the moment before the first frame arrives.
  let srvCfg = { channel: 12, lock_us: 1100, release_us: 1410,
                 min_us: 500, max_us: 2500, deg_span: 180 };
  let plKind = "parcel";
  const auxName = () => (srvCfg.channel > 8 ? `AUX${srvCfg.channel - 8}` : `CH${srvCfg.channel}`);

  $("srv-lock").onclick = () => send("set_servo", { channel: srvCfg.channel, pwm: srvCfg.lock_us });
  $("srv-release").onclick = () => confirmDanger("RELEASE PAYLOAD?",
    `The ${auxName()} servo will swing open and drop whatever is attached.` +
    (plKind === "medbox"
      ? " A hand release opens no MedBox drop window - the box will log the landing as an impact" +
        " (use a delivery drop to hand it over intact)."
      : ""),
    () => send("set_servo", { channel: srvCfg.channel, pwm: srvCfg.release_us }));

  $("mb-reset").onclick = () => confirmDanger("RESET MEDBOX?",
    (mbLast.st === "COMPROMISED"
      ? `This clears the COMPROMISED verdict (${mbLast.reason}${mbLast.value ? ", " + mbLast.value : ""}). ` +
        "Only do it after inspecting the contents - the box forgets the reason (the GCS console keeps it). "
      : `The box is ${mbLast.st}; this returns it to UNARMED. `) +
    "It re-arms 3 s after the lid is closed.",
    () => send("medbox_reset"));

  // Payload kind selector: the hub persists it; the frame echoes it back.
  document.querySelectorAll("#pl-kind [data-kind]").forEach((b) => {
    b.onclick = () => { if (b.dataset.kind !== plKind) send("set_payload_kind", { kind: b.dataset.kind }); };
  });

  // Adopt the served payload config on every frame, so Lock/Release always use
  // what config says (including after a GCS restart) without a page refresh.
  // The manual tuning slider that needed an apply-once guard was removed
  // 2026-09-23.
  srvAdoptCfg = (d) => {
    const pl = d && d.payload;
    if (pl) srvCfg = Object.assign({}, srvCfg, pl);
    if (d && d.payload_kind) plKind = d.payload_kind;
    $("pl-title").textContent = `PAYLOAD · ${auxName()}`;
    document.querySelectorAll("#pl-kind [data-kind]").forEach((b) =>
      b.classList.toggle("on", b.dataset.kind === plKind));
    updateMedbox(d, plKind);
  };

  // --- AUX6 servo (MG90S on FC output SERVO14): two fixed positions --------
  // Not the payload release - a separate mechanism with its own config block
  // (aux2_servo) and its own envelope on the hub.
  //
  // This replaced a full-sweep slider on 2026-09-11. The slider existed only
  // because the servo's working angles were unknown; they are known now, so
  // the UI is two buttons and the horn cannot be parked at an arbitrary angle
  // by accident.
  //
  // Both angles and the channel come from the hub's aux2_servo block. Never
  // hardcode them here: that is exactly the three-way drift docs/PROJECT_NOTES.md §6
  // documents. The values below are a fallback for the moment before the
  // first frame arrives.
  let aux2Cfg = { channel: 14, min_us: 500, max_us: 2500, deg_span: 180,
                  down_deg: 148, up_deg: 90 };
  const aux2DegToUs = (deg) =>
    Math.round(aux2Cfg.min_us + deg * (aux2Cfg.max_us - aux2Cfg.min_us) / aux2Cfg.deg_span);

  const aux2Send = (deg, label) => {
    const us = aux2DegToUs(deg);
    send("set_servo", { channel: aux2Cfg.channel, pwm: us });
    nlLog("servo " + label + " → " + deg + "° (" + us + "µs)", true);
  };
  const aux2Down = $("aux2-down"), aux2Up = $("aux2-up");
  if (aux2Down) aux2Down.onclick = () => aux2Send(aux2Cfg.down_deg, "DOWN");
  if (aux2Up) aux2Up.onclick = () => aux2Send(aux2Cfg.up_deg, "UP");

  // Adopt the served config once, so the buttons command and display the
  // configured angles. Nothing is commanded on load - opening the GCS must
  // not move a mechanism.
  let aux2CfgApplied = false;
  aux2AdoptCfg = (d) => {
    const a2 = d && d.aux2_servo;
    if (!a2 || aux2CfgApplied) return;
    aux2CfgApplied = true;
    aux2Cfg = Object.assign({}, aux2Cfg, a2);
    if (aux2Down) aux2Down.textContent = "DOWN " + Math.round(aux2Cfg.down_deg) + "°";
    if (aux2Up) aux2Up.textContent = "UP " + Math.round(aux2Cfg.up_deg) + "°";
  };

  // TEMP tuning control: emergency-braking trigger distance. The number field and
  // the slider stay in sync and both push set_stop_distance live (throttled for
  // the slider). Once tuned, the exact value goes into avoidance_stop_m in config.
  const brakeNum = $("brake-dist-num"), brakeSlider = $("brake-dist");
  if (brakeNum && brakeSlider) {
    let brakeLast = 0;
    const brakeSend = (v, throttle) => {
      v = Math.max(0.3, Math.min(8, Number(v) || 0));
      brakeNum.value = v; brakeSlider.value = v;
      const now = performance.now();
      if (!throttle || now - brakeLast > 80) {
        send("set_stop_distance", { distance_m: v });
        brakeLast = now;
      }
    };
    brakeSlider.oninput = () => brakeSend(brakeSlider.value, true);
    brakeSlider.onchange = () => brakeSend(brakeSlider.value, false);
    brakeNum.onchange = () => brakeSend(brakeNum.value, false);
  }
  // Flight altitude (takeoff + delivery legs). Deliberately NOT like the brake
  // slider above: that one streams values while dragging, which is fine for a
  // threshold but would make the aircraft chase the thumb. This one sends once,
  // on release, asks first while armed, and otherwise just mirrors what the
  // navigator reports it is actually using (already clamped to the ceiling).
  const altNum = $("flight-alt-num"), altSlider = $("flight-alt"), altNote = $("flight-alt-note");
  if (altNum && altSlider && altNote) {
    const ALT_MIN = 2, ALT_MAX = 10;
    let server = 0, ceil = 0, armed = false, phase = "IDLE";
    let editing = false, editTimer = null;
    const quant = (v) => Math.round(Math.max(ALT_MIN, Math.min(ALT_MAX, Number(v) || 0)) * 2) / 2;
    const show = (v) => { altNum.value = v; altSlider.value = v; };
    const noteFor = (v) => {
      const parts = [];
      if (server) parts.push(`in use: ${fmt(server, 1)} m`);
      if (ceil) parts.push(`ceiling ${fmt(ceil, 1)} m`);
      parts.push("person scan keeps its own fixed height");
      let txt = parts.join(" · ");
      if (ceil && v > ceil + 1e-6) {
        txt = `⚠ ${fmt(v, 1)} m is above the ${fmt(ceil, 1)} m ceiling - it will fly at ${fmt(ceil, 1)} m. ` + txt;
      }
      return txt;
    };
    const holdEdit = () => {                 // stop state updates fighting the user
      editing = true;
      clearTimeout(editTimer);
      editTimer = setTimeout(() => { editing = false; if (server) show(server); }, 8000);
    };
    const commit = (raw) => {
      const v = quant(raw);
      show(v);
      if (server && Math.abs(v - server) < 0.01) { editing = false; altNote.textContent = noteFor(v); return; }
      if (armed) {
        const q = `Change flight altitude to ${fmt(Math.min(v, ceil || v), 1)} m now?\n\n` +
          `Phase: ${phase}. On a delivery leg the drone climbs/descends to it immediately; ` +
          `during takeoff, the scan or a drop it applies afterwards.`;
        if (!window.confirm(q)) { editing = false; if (server) show(server); altNote.textContent = noteFor(server); return; }
      }
      altNote.textContent = "sending…";
      send("set_flight_altitude", { alt_m: v });
    };
    altSlider.oninput = () => { holdEdit(); altNum.value = altSlider.value; altNote.textContent = noteFor(Number(altSlider.value)); };
    altSlider.onchange = () => commit(altSlider.value);
    altNum.onfocus = holdEdit;
    altNum.onchange = () => commit(altNum.value);
    altAdoptState = (d) => {
      const m = (d && d.mission) || {}, t = (d && d.telemetry) || {};
      ceil = Number(m.alt_ceiling_m) || 0;
      armed = !!t.armed;
      phase = m.phase || "IDLE";
      const s = Number(m.flight_alt_m) || 0;
      if (s && s !== server) server = s;
      if (!editing && server) { show(server); altNote.textContent = noteFor(server); }
    };
    altAckHook = (res) => {
      editing = false;
      clearTimeout(editTimer);
      altNote.textContent = (res.ok ? "✓ " : "✗ ") + (res.message || "no response");
      if (!res.ok && server) show(server);
    };
  }
  $("btn-savescan").onclick = saveScan;
  $("btn-replay").onclick = () => { const f = $("log-select").value; if (f) send("replay", { file: f }); };
  $("log-select").onchange = (e) => { if (e.target.value) send("replay", { file: e.target.value }); };
}

window.addEventListener("DOMContentLoaded", () => {
  initTheme(); initDensity(); buildModeGrid(); initMap(); buildSparks(); setupRadarControls();
  wire(); initDelivery(); initCams(); initRecording(); loadLogList(); connect();
});
