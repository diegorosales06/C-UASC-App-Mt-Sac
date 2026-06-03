#!/bin/bash
DEVICE_PATH="/sdcard/Android/data/com.dji.sdk.sample/files"
DEST=${1:-~/Desktop}
echo "Checking for connected device..."
adb devices
echo ""
echo "Files on device:"
adb shell ls "$DEVICE_PATH"
echo ""
echo "Pulling latest flight log to $DEST..."
LATEST=$(adb shell ls -t "$DEVICE_PATH/flight_log_"*.csv 2>/dev/null | head -1 | tr -d '\r')
if [ -z "$LATEST" ]; then
  echo "No flight log files found on device."
  exit 1
fi
adb pull "$LATEST" "$DEST"
echo "Done! Saved to $DEST/$(basename $LATEST)"
