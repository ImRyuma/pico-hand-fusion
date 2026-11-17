#!/system/bin/sh
# Keeps the Motion Trackers at full frequency while the Hand Fusion layer drives a hand.
#
# The layer runs inside PICO Connect without root, so it cannot call the tracking
# service itself. It writes a status file every second; while that file is fresh and
# reports using_trackers=1, this daemon keeps a full-frequency hold alive with
# pvrtracking_swift_test (the hold lasts as long as that process lives).

MODDIR=${0%/*}
DIR=/data/data/com.picovr.picostreamassistant/files
STATUS=$DIR/hand_fusion_status.txt
CONF=$DIR/hand_fusion.conf
BIN=/system/bin/pvrtracking_swift_test
PIDF=$MODDIR/.fullfreq.pid
STATE=$MODDIR/.fullfreq.state
LOG=$MODDIR/fullfreq.log

FRESH_S=5        # status older than this: the layer is not running a hand session
LINGER_S=30      # keep holding this long after the last fresh status
HOLD_MS=3600000  # each hold process re-arms itself for up to one hour
POLL_S=2

if [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then
  exit 0
fi
echo $$ > "$PIDF"

log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

HPID=""
hold_alive() { [ -n "$HPID" ] && kill -0 "$HPID" 2>/dev/null; }
hold_stop() {
  if hold_alive; then kill "$HPID" 2>/dev/null; wait "$HPID" 2>/dev/null; log "hold released"; fi
  HPID=""
}
hold_start() {
  hold_stop
  "$BIN" setKeepTrackerFullFrequencyFlag 1 "$HOLD_MS" >/dev/null 2>&1 &
  HPID=$!
  log "hold started (pid $HPID)"
}
cleanup() { hold_stop; rm -f "$PIDF" "$STATE"; exit 0; }
trap cleanup INT TERM

[ -x "$BIN" ] || { log "missing $BIN, exiting"; rm -f "$PIDF"; exit 0; }
log "started (pid $$)"

last_active=0
while true; do
  now=$(date +%s)
  enabled=1
  if [ -f "$CONF" ]; then
    v=$(sed -n 's/^full_frequency=\([0-9]*\).*/\1/p' "$CONF" | tail -n 1)
    [ -n "$v" ] && enabled=$v
  fi

  active=0
  if [ -f "$STATUS" ]; then
    mtime=$(stat -c %Y "$STATUS" 2>/dev/null)
    : "${mtime:=0}"
    if [ $((now - mtime)) -le $FRESH_S ] && grep -q '^using_trackers=1' "$STATUS"; then
      active=1
      last_active=$now
    fi
  fi

  if [ "$enabled" = "1" ] && { [ "$active" = "1" ] || [ $((now - last_active)) -le $LINGER_S ]; } && [ "$last_active" -gt 0 ]; then
    hold_alive || hold_start
    want=1
  else
    hold_stop
    want=0
  fi
  echo "hold=$want active=$active enabled=$enabled updated=$(date '+%T')" > "$STATE"
  sleep $POLL_S
done
