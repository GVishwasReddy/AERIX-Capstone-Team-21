#!/usr/bin/env bash
# Boot wrapper for the BLE delivery-handshake peripheral.
#
# Run by aerix-ble.service. Three jobs, in order:
#
#   1. Wait for a Bluetooth adapter. bluetooth.service being "active" only
#      means the daemon started; hci0 can still be down for a second or two
#      after boot, and registering an advertisement against a down adapter
#      fails outright.
#   2. Try to pull the current DISPATCHED order from Firestore, which writes
#      active_order.json with that order's one-time deliveryToken. Failure is
#      not fatal: there may simply be no order waiting.
#   3. Start the peripheral.
#
# On (3): --dev is a *fallback*, not an override. drone_ble_peripheral.py uses
# active_order.json whenever the file exists and only falls back to the
# hardcoded bench token when it does not. Passing --dev therefore means "come
# up advertising even with no order pending" - a real order still gets a real
# per-order token. Set AERIX_BLE_STRICT=1 in the unit to drop the fallback and
# refuse to advertise without a genuine order.
set -uo pipefail

BLE_DIR="/home/pi/drone_stack/drone_stack/ble_handshake"
PY="${BLE_DIR}/venv/bin/python"
CRED="/home/pi/drone_stack/config/firebase-service-account.json"
STRICT="${AERIX_BLE_STRICT:-0}"

cd "${BLE_DIR}" || exit 1

# The order token lives in RAM, never on the SD card (order_store.py,
# 2026-10-08). Export the directory so the pull below and the peripheral
# agree on it. If it cannot be created, order_store falls back to the card
# with the token still sealed to this board - and says so in the log.
ORDER_DIR="${AERIX_ORDER_DIR:-/run/aerix-ble}"
if mkdir -p -m 700 "${ORDER_DIR}" 2>/dev/null && [ -w "${ORDER_DIR}" ]; then
    export AERIX_ORDER_DIR="${ORDER_DIR}"
    echo "order token directory: ${ORDER_DIR} (RAM)"
else
    echo "WARNING: cannot use ${ORDER_DIR}; the order token will be sealed but on the SD card"
fi

# -- 1. wait for an adapter --------------------------------------------------
# The handshake runs on the ON-BOARD controller (the peripheral pins itself to
# it; the USB dongle belongs to the MedBox bridge, aerix-payload.service). So
# wait for that one specifically - "any adapter UP" is satisfied by the dongle
# alone and would let the peripheral register against a still-down hci.
onboard_up() {
    local h
    for h in /sys/class/bluetooth/hci*; do
        [ -e "$h" ] || continue
        case "$(readlink -f "$h")" in *usb*) continue ;; esac
        hciconfig "$(basename "$h")" 2>/dev/null | grep -q "UP RUNNING" && return 0
    done
    return 1
}
for _ in $(seq 1 30); do
    if onboard_up; then
        break
    fi
    echo "waiting for the on-board Bluetooth adapter to come up..."
    sleep 2
done
onboard_up || echo "WARNING: on-board adapter not UP after 60s - the peripheral will use the first adapter BlueZ offers"

if ! hciconfig 2>/dev/null | grep -q "UP RUNNING"; then
    echo "no Bluetooth adapter is UP RUNNING after 60s - giving up this attempt"
    exit 1          # systemd Restart=always will try again
fi
echo "adapter ready:"
hciconfig 2>/dev/null | grep -E "^hci|BD Address"

# -- 2. try for a real order -------------------------------------------------
if [ -f "${CRED}" ]; then
    echo "checking Firestore for a DISPATCHED order..."
    if "${PY}" firebase_sync.py --cred "${CRED}" pull-order; then
        echo "pulled an active order"
    else
        echo "no DISPATCHED order available (this is normal when idle)"
    fi
else
    echo "no service-account key at ${CRED} - cannot pull a per-order token"
fi

# -- 3. start the peripheral -------------------------------------------------
# The peripheral decides, not this script. It uses active_order.json whenever
# that file exists AND is fresh enough to still be the token the app holds, and
# falls back to the bench token otherwise - so --dev means "come up advertising
# even without a usable order", never "ignore the order".
#
# Deciding here as well was a bug: this tested only whether the file EXISTED, so
# a stale order file selected secure mode and the peripheral was then left with
# no fallback to take. On 2026-08-30 that served a token pulled three days
# earlier and every handshake failed HMAC authentication.
if [ "${STRICT}" = "1" ]; then
    echo "AERIX_BLE_STRICT=1 - a real, current order is required (no bench fallback)"
    exec "${PY}" -u drone_ble_peripheral.py
fi

exec "${PY}" -u drone_ble_peripheral.py --dev
