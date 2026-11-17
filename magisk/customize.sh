#!/system/bin/sh
# Magisk installer hook for Pico Hand Fusion.

SKIPUNZIP=0

if [ "$ARCH" != "arm64" ]; then
  abort "! This module only supports arm64 (Pico 4 / Pico 4 Pro)"
fi

if [ ! -f "$MODPATH/zygisk/arm64-v8a.so" ]; then
  abort "! zygisk/arm64-v8a.so is missing from the zip (build it first)"
fi

if [ "$(magisk --sqlite "select value from settings where key='zygisk'" 2>/dev/null)" != "value=1" ]; then
  ui_print "! Zygisk is disabled: enable it in the Magisk app settings, then reboot"
fi

ui_print "- Installing the hand fusion (Zygisk, PICO Connect streaming process only)"
set_perm "$MODPATH/fullfreq.sh" 0 0 0755
set_perm "$MODPATH/service.sh" 0 0 0755

# Install the companion app when the zip is flashed from the running system.
if [ -f "$MODPATH/companion.apk" ]; then
  if pm install -r "$MODPATH/companion.apk" >/dev/null 2>&1; then
    ui_print "- Companion app installed"
  else
    ui_print "- Companion app not installed now; run: adb install companion.apk"
  fi
fi

ui_print "- Reboot, then put the hand trackers in Object tracking mode"
