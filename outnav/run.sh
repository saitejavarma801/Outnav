#!/bin/bash
# --- Prevent OpenCV from loading its own Qt plugin path ---
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH
export QT_QPA_PLATFORM=xcb
export QTWEBENGINE_DISABLE_SANDBOX=1
export QTWEBENGINE_CHROMIUM_FLAGS="--no-sandbox --disable-gpu --disable-dev-shm-usage --autoplay-policy=no-user-gesture-required"

# Optional ROS2 middleware/domain overrides
if [ -n "${OUTNAV_RMW}" ]; then
  export RMW_IMPLEMENTATION="${OUTNAV_RMW}"
fi
if [ -n "${OUTNAV_DDS}" ]; then
  export RMW_IMPLEMENTATION="${OUTNAV_DDS}"
fi
if [ -n "${OUTNAV_DOMAIN_ID}" ]; then
  export ROS_DOMAIN_ID="${OUTNAV_DOMAIN_ID}"
fi

# Run the app (headless robot mode if requested)
if [ "${OUTNAV_HEADLESS}" = "1" ] || [ "${OUTNAV_HEADLESS}" = "true" ]; then
  python3 robot_headless.py
else
  python3 app.py
fi
