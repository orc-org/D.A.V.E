#!/bin/bash

# default configurations
PORT=5000
CODEC="h264"
DIAGNOSTICS=false

# help message
show_help() {
    echo "Low-Latency Video Stream Receiver Script"
    echo "========================================"
    echo "Usage: ./receive_stream.sh [options]"
    echo ""
    echo "Options:"
    echo "  -p, --port <number>     UDP port to listen on (default: 5000)"
    echo "  -m, --mjpeg             Expect an MJPEG stream (default: H.264)"
    echo "  -d, --diagnostics       Show real-time FPS and latency diagnostics overlay"
    echo "  -h, --help              Show this help message"
    echo ""
}

# Parse options
while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--port)
            # without this check "-p" as the last argument made 'shift 2' fail and the loop spin forever
            if [[ $# -lt 2 || ! "$2" =~ ^[0-9]+$ ]]; then
                echo "Option $1 needs a numeric port"
                exit 1
            fi
            PORT="$2"
            shift 2
            ;;
        -m|--mjpeg)
            CODEC="mjpeg"
            shift 1
            ;;
        -d|--diagnostics)
            DIAGNOSTICS=true
            shift 1
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "Starting low-latency video stream receiver"
echo "Listening Port: $PORT"
echo "Expected Codec: $CODEC"
echo "Diagnostics:    $DIAGNOSTICS"
echo "=========================================="

command -v gst-launch-1.0 >/dev/null 2>&1 || { echo "gst-launch-1.0 not found (sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-libav)"; exit 1; }

# Build the pipeline as an argument array. The previous version joined it into a string and
# ran it through eval, which broke on the quoted video-sink value and on odd input.
if [ "$CODEC" = "h264" ]; then
    CAPS="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"
    DECODER=(rtph264depay ! h264parse ! avdec_h264)
else
    CAPS="application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26"
    DECODER=(rtpjpegdepay ! jpegdec)
fi

if [ "$DIAGNOSTICS" = true ]; then
    # fpsdisplaysink overlays the frame rate
    # (no spaces inside an argument: gst-launch would escape them and mis-parse the property)
    SINK=(fpsdisplaysink video-sink=autovideosink sync=false)
else
    SINK=(autovideosink sync=false)
fi

PIPELINE=(udpsrc port="$PORT" ! "$CAPS" ! rtpjitterbuffer latency=0 drop-on-latency=true ! "${DECODER[@]}" ! videoconvert ! "${SINK[@]}")

echo "Executing: gst-launch-1.0 -v ${PIPELINE[*]}"
echo ""

exec gst-launch-1.0 -v "${PIPELINE[@]}"
