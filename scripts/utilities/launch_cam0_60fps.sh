#!/bin/bash
# Custom script to launch CSI Camera 0 at 60 FPS for high-speed Morse decoding
#
# Environment: HOST  base-station IP that receives the stream (default 192.168.1.87)
#              PORT  UDP port on the base station          (default 5000)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STREAMER="$HERE/../../ros2_ws/src/rover_video_streamer/rover_video_streamer/video_streamer.py"

if [ ! -f "$STREAMER" ]; then
    echo "video_streamer.py not found at $STREAMER" >&2
    exit 1
fi

echo "Launching CSI Camera 0 at 60 FPS..."
exec python3 "$STREAMER" \
    --camera-type csi \
    --device 0 \
    --host "${HOST:-192.168.1.87}" \
    --port "${PORT:-5000}" \
    --fps 60 \
    --width 1280 \
    --height 720
