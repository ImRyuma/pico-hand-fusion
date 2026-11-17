#!/usr/bin/env bash
# Sets hand fusion settings live from the PC: tools/setconf.sh key=value [key=value...]
# The layer reloads the file within a second; the full-frequency daemon within two.
set -euo pipefail
export MSYS_NO_PATHCONV=1
DIR=/data/data/com.picovr.picostreamassistant/files
CONF=$DIR/hand_fusion.conf

for kv in "$@"; do
  case "$kv" in
    *=*) ;;
    *) echo "expected key=value, got '$kv'" >&2; exit 1 ;;
  esac
  key=${kv%%=*}
  adb shell "su -c '
    set -e
    owner=\$(stat -c %u:%g /data/data/com.picovr.picostreamassistant)
    touch $CONF
    grep -v \"^$key=\" $CONF > $CONF.tmp || true
    echo \"$kv\" >> $CONF.tmp
    chown \$owner $CONF.tmp; chmod 0600 $CONF.tmp
    mv $CONF.tmp $CONF; restorecon $CONF
  '" >/dev/null
  echo "set $kv"
done
