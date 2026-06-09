#!/bin/bash
# Camera streaming script for Raspberry Pi
# Streams camera as MJPEG over HTTP on port 8081
# Requires: ffmpeg OR motion

# --- Option 1: FFmpeg (recommended for Raspberry Pi Camera Module) ---
ffmpeg_stream() {
  echo "[INFO] Starting FFmpeg camera stream on port 8081..."
  ffmpeg \
    -f v4l2 -input_format mjpeg -i /dev/video0 \
    -vf scale=640:480 \
    -r 20 \
    -q:v 5 \
    -f mjpeg \
    pipe:1 | \
  nc -l -p 8081 -k
}

# --- Option 2: raspivid + FFmpeg (for legacy Pi Camera) ---
raspicam_stream() {
  echo "[INFO] Starting raspivid camera stream on port 8081..."
  raspivid -o - -t 0 -w 640 -h 480 -fps 20 | \
  ffmpeg -i - -vcodec copy -f mjpeg pipe:1 | \
  nc -l -p 8081 -k
}

# --- Option 3: mjpg-streamer (lightest, best for Pi Zero) ---
# sudo apt install mjpg-streamer
# mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 640x480 -f 20" \
#               -o "output_http.so -p 8081"

ffmpeg_stream
