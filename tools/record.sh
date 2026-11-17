#!/usr/bin/env bash
# Triggers a recording of the hand fusion layer from the PC and pulls the CSV when done.
#   tools/record.sh [seconds] [output.csv]
# Run it while a PICO Connect session with hand tracking is active.
set -euo pipefail
export MSYS_NO_PATHCONV=1

SECONDS_TO_RECORD="${1:-60}"
OUTPUT="${2:-hand_fusion_record.csv}"
DIR=/data/data/com.picovr.picostreamassistant/files
CONF=$DIR/hand_fusion.conf

# Bump record_epoch, keeping the file owned and labeled like the app's own files.
adb shell "su -c '
  set -e
  owner=\$(stat -c %u:%g /data/data/com.picovr.picostreamassistant)
  touch $CONF
  epoch=\$(sed -n \"s/^record_epoch=//p\" $CONF | tail -n 1)
  epoch=\$(( \${epoch:-0} + 1 ))
  grep -v -E \"^(record_epoch|record_seconds)=\" $CONF > $CONF.tmp || true
  echo record_epoch=\$epoch >> $CONF.tmp
  echo record_seconds=$SECONDS_TO_RECORD >> $CONF.tmp
  chown \$owner $CONF.tmp; chmod 0600 $CONF.tmp
  mv $CONF.tmp $CONF; restorecon $CONF
  rm -f $DIR/hand_fusion_record.csv
'"
echo "Recording ${SECONDS_TO_RECORD} s (starts within a second)..."
sleep $((SECONDS_TO_RECORD + 3))
for _ in $(seq 1 20); do
  if adb shell "su -c 'test -s $DIR/hand_fusion_record.csv'" 2>/dev/null; then
    adb shell "su -c 'cat $DIR/hand_fusion_record.csv'" > "$OUTPUT"
    echo "Saved $(wc -l < "$OUTPUT") lines to $OUTPUT"
    exit 0
  fi
  sleep 1
done
echo "No recording found: is a hand tracking session running?" >&2
exit 1
