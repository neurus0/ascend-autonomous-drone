#!/bin/bash
# setup-pi.sh  –  Run this ONCE on your drone's Raspberry Pi
# Sets up all dependencies for bridge.py and camera_stream.py
#
# Usage:
#   chmod +x setup-pi.sh
#   ./setup-pi.sh

set -e

echo ""
echo "  Drone GCS — Raspberry Pi Setup"
echo "  ================================"
echo ""

# Update package list
echo "[1/6] Updating package list…"
sudo apt-get update -qq

# Python & pip
echo "[2/6] Installing Python3 & pip…"
sudo apt-get install -y python3 python3-pip python3-dev

# pymavlink
echo "[3/6] Installing pymavlink…"
pip3 install --upgrade pymavlink

# OpenCV (for camera streaming)
echo "[4/6] Installing OpenCV…"
pip3 install opencv-python-headless   # headless = no GUI dependencies

# FFmpeg (alternative camera method)
echo "[5/6] Installing FFmpeg…"
sudo apt-get install -y ffmpeg

# Serial port permissions
echo "[6/6] Adding user to dialout group (serial port access)…"
sudo usermod -a -G dialout $USER

echo ""
echo "  ✓ Setup complete!"
echo ""
echo "  Next steps:"
echo "  1. Start the MAVLink bridge:"
echo "     python3 bridge.py --gcs-ip <YOUR_LAPTOP_IP>"
echo ""
echo "  2. Start the camera stream:"
echo "     python3 camera_stream.py"
echo ""
echo "  NOTE: Log out and back in for dialout group to take effect."
echo ""
