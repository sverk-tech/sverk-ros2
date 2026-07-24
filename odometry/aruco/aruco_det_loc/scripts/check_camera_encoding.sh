#!/usr/bin/env bash
# check_camera_encoding.sh
# -------------------------
# Checks what image encoding the camera actually publishes so you can decide
# whether to enable the MONO8 zero-copy path in aruco_detect_node.
#
# Usage:
#   bash check_camera_encoding.sh [TOPIC]
#
# TOPIC defaults to the Gazebo sim topic. Pass your real camera topic, e.g.:
#   bash check_camera_encoding.sh /camera/image_raw
#
# The script also checks V4L2 device capabilities if available.

set -euo pipefail

TOPIC="${1:-/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image}"

echo ""
echo "=== Camera Encoding Check ==="
echo "Topic: ${TOPIC}"
echo ""

# ---- ROS2 topic encoding ----
echo "--- Checking ROS2 image encoding (requires active camera publisher) ---"
echo "(Waiting up to 10 seconds for a message...)"
echo ""

if ros2 topic echo --once --no-arr "${TOPIC}" 2>/dev/null | grep -E "encoding|width|height|step" | head -10; then
    echo ""
else
    echo "⚠ Could not read from topic '${TOPIC}'."
    echo "  Make sure the camera driver / simulation is running."
    echo ""
fi

# ---- Summarise what we look for ----
echo "--- What to look for ---"
echo ""
echo "  encoding: 'mono8'  or '8UC1'  → MONO8 path available (zero-copy)"
echo "  encoding: 'bgr8'   or 'rgb8'  → BGR path (requires COLOR_BGR2GRAY)"
echo "  encoding: 'mono16'            → 16-bit mono (aruco_detect converts to MONO8 on load)"
echo "  encoding: 'bayer_*'           → Bayer pattern (not directly supported; use ISP)"
echo ""

# ---- V4L2 device info ----
if command -v v4l2-ctl &>/dev/null; then
    echo "--- V4L2 supported formats (physical camera) ---"
    v4l2-ctl --list-formats-ext 2>/dev/null || echo "(No V4L2 device found or no permission)"
    echo ""
else
    echo "--- v4l2-ctl not found (install v4l-utils to check physical camera formats) ---"
    echo ""
fi

# ---- Recommendation ----
echo "--- Recommendation ---"
echo ""
echo "IMX219 in Raspberry Pi camera mode can output:"
echo "  • RAW10 Bayer (SRGGB10) — requires ISP, not directly usable"
echo "  • YUV (YUYV) — convert with v4l2_camera to mono8 before publishing"
echo "  • Use libcamera or ros2-camera-driver with output_encoding:=mono8"
echo ""
echo "Gazebo simulation: check sim bridge encoding."
echo "If the topic publishes 'mono8' or '8UC1', set no extra parameter —"
echo "aruco_detect_node will detect it automatically and use zero-copy."
echo ""
echo "=== Done ==="
