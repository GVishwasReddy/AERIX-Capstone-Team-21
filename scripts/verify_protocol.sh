#!/usr/bin/env bash
# Re-run the formal models of the BLE release handshake and save dated output.
#
#   scripts/verify_protocol.sh          # Verifpal on all four models
#   PROVERIF=1 scripts/verify_protocol.sh   # also ProVerif on the .pv model
#
# Install (Mac):  brew install rust && cargo install verifpal   (or build from
#                 github.com/symbolicsoft/verifpal);  opam install proverif
# Expected (2026-10-08, Verifpal 1.6.4, bounded at 2 sessions):
#   aerix_handshake.vp              4 of 6 fail  - today's app: phone signs any challenge (T29)
#   aerix_handshake_v1_1.vp         3 of 6 fail  - only replays TO the phone remain (T22)
#   aerix_handshake_v2.vp           all 7 pass   - labels + phone nonce
#   aerix_handshake_plain_result.vp 4 of 6 fail  - control: forged RESULT must be found
set -uo pipefail
cd "$(dirname "$0")/.."
OUT=docs/security/verifpal_results
mkdir -p "$OUT"
DAY=$(date +%F)
for m in aerix_handshake aerix_handshake_v1_1 aerix_handshake_v2 aerix_handshake_plain_result; do
    f="$OUT/${DAY}_${m#aerix_handshake_}.txt"
    [ "$m" = aerix_handshake ] && f="$OUT/${DAY}_current.txt"
    verifpal verify "docs/security/$m.vp" > "$f" 2>&1
    printf '%-34s %s\n' "$m" "$(grep -E 'queries (pass|failed)' "$f" | tail -1 | sed 's/^ *//')"
done
if [ "${PROVERIF:-0}" = 1 ]; then
    proverif docs/security/aerix_handshake.pv > "$OUT/${DAY}_proverif.txt" 2>&1
    grep -E "^RESULT" "$OUT/${DAY}_proverif.txt"
fi
