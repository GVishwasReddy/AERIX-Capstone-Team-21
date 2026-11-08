"""Where the active order's single-use token lives, and how it is kept.

Why this exists - the "captured drone" gap (2026-10-08 evidence review against
SecAuthUAV): to check the phone's HMAC the drone must hold the order token, and
``firebase_sync.py pull-order`` wrote it in the clear to
``ble_handshake/active_order.json`` on the SD card. Pull the card, image it,
read the token - and with it forge the phone's side of the handshake for that
order. The spent markers already hold no token (``_invalidate_order_file``);
this closes the live file, in two independent layers:

  1. RAM, not flash. The order file defaults to ``/run/aerix-ble/`` - tmpfs on
     Raspberry Pi OS - so the token never reaches the card at all and is gone
     at power-off. Deleting a file does not scrub flash; never writing it does.
     ``AERIX_ORDER_DIR`` overrides; a non-root dev run falls back to
     ``$XDG_RUNTIME_DIR`` (also tmpfs) and only then to the legacy directory,
     with a warning.
  2. Sealed. Wherever the file lands, the token inside it is AES-256-GCM
     encrypted (``deliveryTokenSealed``) under a key derived (HKDF-SHA256)
     from the board's SoC serial number - burned into the chip, not stored on
     the card - with the order id as associated data, so a sealed token cannot
     be pasted into another order's file either.

What it does NOT do, stated so nobody claims more: the SoC serial is a
*binding*, not a secret. An attacker holding the whole powered-off aircraft
can read the serial and unseal a file that was on flash - which is why layer 1
keeps it off flash. An attacker with root on the RUNNING aircraft can read the
token out of the peripheral's memory; only a secure element / TPM holding the
key (or a PUF, as SecAuthUAV uses) closes that, and this board has neither.
See docs/security/THREAT_MODEL.md, rows T19-T21.

The spent markers are deliberately NOT moved to RAM: they hold no token, and
they are what stops a spent order's token being re-accepted after a reboot.
``spent_dir()`` keeps them on persistent storage.
"""
from __future__ import annotations

import base64
import json
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
LEGACY_DIR = _HERE
ORDER_NAME = "active_order.json"
SYSTEM_RUN_DIR = Path("/run/aerix-ble")

SEAL_VERSION = 1
_HKDF_INFO = b"AERIX/order-token-seal/v1"
_HKDF_SALT = b"AERIX-order-store"

# Filesystems whose contents never reach persistent storage.
_VOLATILE_FS = {"tmpfs", "ramfs"}


# ── where ───────────────────────────────────────────────────────────────────

def _usable_dir(path: Path) -> bool:
    try:
        path.mkdir(mode=0o700, parents=True, exist_ok=True)
    except OSError:
        return False
    return os.access(path, os.W_OK | os.X_OK)


def order_dir() -> Path:
    """The directory the active order file is written to and read from."""
    env = os.environ.get("AERIX_ORDER_DIR")
    if env:
        return Path(env)
    if _usable_dir(SYSTEM_RUN_DIR):
        return SYSTEM_RUN_DIR
    xdg = os.environ.get("XDG_RUNTIME_DIR")
    if xdg:
        cand = Path(xdg) / "aerix-ble"
        if _usable_dir(cand):
            return cand
    return LEGACY_DIR


def default_order_path() -> Path:
    return order_dir() / ORDER_NAME


def fs_type(path: Path) -> str:
    """Filesystem type holding ``path`` (longest /proc/mounts prefix), or ""."""
    try:
        target = os.path.realpath(path)
        best, kind = "", ""
        with open("/proc/mounts", encoding="utf-8") as fh:
            for line in fh:
                parts = line.split()
                if len(parts) < 3:
                    continue
                mnt = parts[1].replace("\\040", " ")
                if (target == mnt or target.startswith(mnt.rstrip("/") + "/")
                        or mnt == "/") and len(mnt) >= len(best):
                    best, kind = mnt, parts[2]
        return kind
    except OSError:
        return ""


def is_volatile(path: Path) -> bool:
    """Does ``path`` live on RAM (tmpfs/ramfs) rather than the SD card?"""
    return fs_type(path) in _VOLATILE_FS


def spent_dir(order_file: Path) -> Path:
    """Where token-free spent markers go: next to the order file, unless that
    is RAM - then the persistent legacy directory, so they survive a reboot
    (and scripts/verify_receipts.py still finds them where it always did)."""
    env = os.environ.get("AERIX_SPENT_DIR")
    if env:
        return Path(env)
    parent = Path(order_file).parent
    return LEGACY_DIR if is_volatile(parent) else parent


# ── sealing ─────────────────────────────────────────────────────────────────

def _board_serial() -> tuple[bytes, str]:
    """Stable per-board material that is NOT stored on the SD card."""
    override = os.environ.get("AERIX_SEAL_SECRET")
    if override:
        return override.encode("utf-8"), "env"
    try:
        raw = Path("/sys/firmware/devicetree/base/serial-number").read_bytes()
        raw = raw.strip(b"\x00\n ")
        if raw:
            return raw, "soc-serial"
    except OSError:
        pass
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            if line.lower().startswith("serial"):
                val = line.split(":", 1)[1].strip()
                if val.strip("0"):
                    return val.encode("ascii"), "soc-serial"
    except OSError:
        pass
    # Not a Pi (dev machine / CI). machine-id IS on disk, so this seals against
    # copying the file to another host, not against imaging this one.
    for p in ("/etc/machine-id", "/var/lib/dbus/machine-id"):
        try:
            val = Path(p).read_bytes().strip()
            if val:
                return val, "machine-id"
        except OSError:
            continue
    return b"AERIX-no-board-identity", "none"


def seal_binding() -> str:
    """What the seal is bound to: soc-serial | machine-id | env | none."""
    return _board_serial()[1]


def _key() -> bytes:
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.kdf.hkdf import HKDF
    material, _ = _board_serial()
    return HKDF(algorithm=hashes.SHA256(), length=32, salt=_HKDF_SALT,
                info=_HKDF_INFO).derive(material)


def seal_token(token: bytes, order_id: str) -> str:
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    nonce = os.urandom(12)
    ct = AESGCM(_key()).encrypt(nonce, bytes(token), order_id.encode("utf-8"))
    return f"v{SEAL_VERSION}:" + base64.b64encode(nonce + ct).decode("ascii")


def unseal_token(sealed: str, order_id: str) -> bytes:
    """Raises ValueError if the seal is malformed, from another board, or was
    moved to a different order id."""
    from cryptography.exceptions import InvalidTag
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    ver, _, body = sealed.partition(":")
    if ver != f"v{SEAL_VERSION}" or not body:
        raise ValueError(f"unknown token seal format {ver!r}")
    blob = base64.b64decode(body)
    try:
        return AESGCM(_key()).decrypt(blob[:12], blob[12:], order_id.encode("utf-8"))
    except InvalidTag as exc:
        raise ValueError("sealed token does not open on this board / for this "
                         "order (copied file or tampered)") from exc


# ── read / write ────────────────────────────────────────────────────────────

def write_order(path: Path, order: dict) -> Path:
    """Write ``order`` (with a plain ``deliveryToken`` hex) to ``path`` with the
    token sealed, owner-only (0600), atomically. Returns the path written."""
    path = Path(path)
    out = {k: v for k, v in order.items() if k != "deliveryToken"}
    token = bytes.fromhex(str(order["deliveryToken"]))
    try:
        out["deliveryTokenSealed"] = seal_token(token, str(order["orderId"]))
        out["sealBinding"] = seal_binding()
    except ImportError:
        # The BLE venv always has `cryptography` (receipt_ledger needs it).
        # Anywhere else, RAM + 0600 is still better than the old behaviour.
        print("  ⚠️ [ORDER] `cryptography` missing - token stored UNSEALED "
              "(still owner-only).", file=sys.stderr)
        out["deliveryToken"] = token.hex()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    fd = os.open(str(tmp), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        os.write(fd, json.dumps(out, indent=2).encode("utf-8"))
        os.fsync(fd)
    finally:
        os.close(fd)
    os.replace(tmp, path)
    if not is_volatile(path.parent):
        print(f"  ⚠️ [ORDER] {path} is on persistent storage, not RAM - the "
              "token is sealed but set AERIX_ORDER_DIR to a tmpfs.", file=sys.stderr)
    return path


def read_order(path: Path) -> dict:
    """Load an order file; ``deliveryToken`` (hex) is always present in the
    result, unsealed if necessary. Legacy plain files still load."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if "deliveryTokenSealed" in data:
        data["deliveryToken"] = unseal_token(
            data["deliveryTokenSealed"], str(data["orderId"])).hex()
    if "deliveryToken" not in data:
        raise KeyError("deliveryToken")
    return data


def token_forms(token: bytes) -> dict[str, bytes]:
    """Every encoding the stack has ever stored a token in - what
    scripts/token_capture_check.py searches a card image for."""
    token = bytes(token)
    return {
        "raw": token,
        "hex": token.hex().encode(),
        "HEX": token.hex().upper().encode(),
        "base64": base64.b64encode(token).rstrip(b"="),
    }
