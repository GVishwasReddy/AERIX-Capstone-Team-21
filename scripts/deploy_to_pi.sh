#!/usr/bin/env bash
# Copy this repo (the Mac mirror) onto the Pi's ~/drone_stack - safely.
#
#   scripts/deploy_to_pi.sh            # DRY RUN: show what would change
#   scripts/deploy_to_pi.sh --apply    # back up, copy, install, test, restart
#
# Why not "replace the Pi's folder": ~/drone_stack on the Pi holds things git
# never had and the Mac mirror does not carry - the venvs, the .hef models,
# the Firebase service-account key, the drone's Ed25519 receipt key, the
# receipt ledger, spent-order markers, logs and recordings. Replacing the
# folder deletes them; losing the receipt key or ledger is unrecoverable.
# This script copies source only, never deletes on the Pi, and backs up
# what it overwrites.
set -euo pipefail

PI="${PI:-pi@pi.local}"
DEST="${DEST:-/home/pi/drone_stack}"
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1
cd "$(dirname "$0")/.."

# Runtime state that lives only on the Pi. Never sent, never deleted.
EXCLUDES=(
  --exclude .git/ --exclude .venv/ --exclude venv/ --exclude __pycache__/
  --exclude '*.pyc' --exclude .pytest_cache/
  --exclude 'models/*.hef' --exclude 'person_detect/*.hef'
  --exclude 'config/firebase-service-account.json' --exclude 'config/pending_order.json'
  --exclude 'drone_stack/ble_handshake/keys/' --exclude 'drone_stack/ble_handshake/receipts/'
  --exclude 'drone_stack/ble_handshake/active_order*.json'
  --exclude 'drone_stack/ble_handshake/venv/'
  --exclude logs/ --exclude recordings/ --exclude graphify-out/ --exclude .synctmp/
  --exclude '*.predeploy_*' --exclude '*.preyaw_*' --exclude '*.bw_*'
  --exclude '.claude_backup_*/' --exclude err.txt --exclude hailort.log
)

echo "== Pi-side edits to config/real.yaml that this copy would overwrite:"
if ssh "$PI" "cat '$DEST/config/real.yaml'" > /tmp/real.yaml.pi 2>/dev/null; then
  if diff -u /tmp/real.yaml.pi config/real.yaml > /tmp/real.yaml.diff; then
    echo "   (none - identical)"
  else
    echo "   see /tmp/real.yaml.diff ('-' lines exist ONLY on the Pi). Carry any"
    echo "   you want to keep into config/real.yaml here before --apply."
    grep -c '^-[^-]' /tmp/real.yaml.diff | sed 's/^/   Pi-only lines: /'
  fi
fi

if [ "$APPLY" = 0 ]; then
  echo; echo "== DRY RUN - files that would be copied (nothing is deleted on the Pi):"
  rsync -avn --checksum "${EXCLUDES[@]}" ./ "$PI:$DEST/" | sed -n '1,200p'
  echo; echo "Run again with --apply to deploy."
  exit 0
fi

STAMP=$(date +%Y%m%d_%H%M%S)
echo "== backing up the Pi's current source to ~/drone_stack_backup_$STAMP"
ssh "$PI" "rsync -a ${EXCLUDES[*]} '$DEST/' ~/drone_stack_backup_$STAMP/"

echo "== stopping services (they hold the serial ports and BLE)"
ssh "$PI" "sudo systemctl stop aerix-gcs.service aerix-ble.service || true"

echo "== copying"
rsync -av --checksum "${EXCLUDES[@]}" ./ "$PI:$DEST/"

echo "== dependencies + tests on the Pi"
ssh "$PI" "cd '$DEST' && \
  drone_stack/ble_handshake/venv/bin/pip install -q -r drone_stack/ble_handshake/requirements.txt && \
  .venv/bin/pip install -q -r requirements.txt && \
  .venv/bin/python -m pytest -q -p no:cacheprovider 2>&1 | tail -3"

echo "== starting services"
ssh "$PI" "sudo systemctl start aerix-ble.service aerix-gcs.service && sleep 15 && \
  systemctl is-active aerix-ble.service aerix-gcs.service; \
  journalctl -u aerix-ble -n 40 --no-pager | grep -E 'order token directory|Traceback|Error' || true"

echo
echo "Deployed. Backup: ~/drone_stack_backup_$STAMP on the Pi."
echo "Rollback: ssh $PI 'rsync -a ~/drone_stack_backup_$STAMP/ $DEST/' and restart the services."
echo "Next: docs/DEPLOY_2026-10-08.md section 2 (receipt reconcile, token scrub)."
