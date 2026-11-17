#!/system/bin/sh
# Remove the files the layer and the companion app created in PICO Connect's data dir.
DIR=/data/data/com.picovr.picostreamassistant/files
rm -f "$DIR/hand_fusion.conf" "$DIR/hand_fusion_status.txt" "$DIR/hand_fusion_status.txt.tmp"
# The companion app (com.ryuma.handfusion) is a regular app: uninstall it from the system settings.
