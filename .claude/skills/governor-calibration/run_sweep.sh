#!/usr/bin/env bash
# Run the embassy-dshot `rpm_range` sweep against one engine and capture its log.
#
# Usage: run_sweep.sh <pin> <normal|reversed> <out.log>
#
# `rpm_range` hardcodes PIN_14 and never sets spin direction, so this patches both
# into the example for the duration of the run and restores the file on exit, even
# on Ctrl-C. The sweep loops forever once done; probe-rs is killed as soon as the
# completion line appears.
set -euo pipefail

PIN=${1:?pin number, e.g. 14}
DIR=${2:?normal|reversed}
OUT=${3:?output log path}
EXAMPLES=${DSHOT_EXAMPLES:-$(cd "$(dirname "$0")/../../../.." && pwd)/dshot-pio/examples}
SRC="$EXAMPLES/src/rpm_range.rs"
TIMEOUT_S=${SWEEP_TIMEOUT_S:-300}

case "$DIR" in
  normal) CMD=SpinDirectionNormal ;;
  reversed) CMD=SpinDirectonReversed ;; # sic: the dshot-frame variant is misspelled
  *) echo "direction must be normal or reversed" >&2; exit 2 ;;
esac

[[ -f "$SRC" ]] || { echo "not found: $SRC (set DSHOT_EXAMPLES)" >&2; exit 2; }
if ! git -C "$EXAMPLES" diff --quiet -- src/rpm_range.rs; then
  echo "$SRC has uncommitted changes; refusing to patch over them" >&2
  exit 2
fi

restore() { git -C "$EXAMPLES" checkout -q -- src/rpm_range.rs; }
trap restore EXIT

grep -q 'p\.PIN_14' "$SRC" || { echo "PIN_14 not found in rpm_range.rs; the example changed" >&2; exit 2; }
grep -q 'info!("ESC armed");' "$SRC" || { echo "arm marker not found in rpm_range.rs; the example changed" >&2; exit 2; }
sed -i "s/p\.PIN_14/p.PIN_${PIN}/" "$SRC"
# Assert direction right after arming, while the ESC is stopped: the same place and
# the same session-only way the elle firmware does it (no SettingsSave).
sed -i "s|info!(\"ESC armed\");|info!(\"ESC armed\");\n    defmt::unwrap!(dshot.send_command_repeated_async(Command::${CMD}, SETTINGS_REPEAT).await);\n    info!(\"Spin direction: ${DIR}\");|" "$SRC"

cd "$EXAMPLES"
cargo build --release --bin rpm_range
: > "$OUT"
cargo run --release --bin rpm_range >"$OUT" 2>&1 &
PID=$!

deadline=$((SECONDS + TIMEOUT_S))
status=0
while kill -0 "$PID" 2>/dev/null; do
  if grep -q 'RPM range test complete!' "$OUT"; then break; fi
  if ((SECONDS >= deadline)); then echo "sweep timed out after ${TIMEOUT_S}s" >&2; status=1; break; fi
  sleep 1
done
kill "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true

grep -q 'RPM range test complete!' "$OUT" || { echo "sweep did not complete; see $OUT" >&2; exit 1; }
echo "sweep complete: $OUT"
exit $status
