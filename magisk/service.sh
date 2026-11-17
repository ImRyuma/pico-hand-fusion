#!/system/bin/sh
# Starts the full-frequency daemon once the system has booted.
MODDIR=${0%/*}

until [ "$(getprop sys.boot_completed)" = "1" ]; do sleep 2; done
sleep 10
nohup sh "$MODDIR/fullfreq.sh" >/dev/null 2>&1 &
