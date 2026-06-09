#!/usr/bin/env python3
"""
camera_stream.py  –  MJPEG Camera Streamer
═══════════════════════════════════════════
Runs ON the drone's Raspberry Pi.

Captures frames from the Pi camera (or USB webcam) and serves them
as an HTTP MJPEG stream that any browser <img> tag can display.

Usage:
  python3 camera_stream.py
  python3 camera_stream.py --port 8081 --width 640 --height 480 --fps 20

Dependencies:
  pip3 install opencv-python

  # For Raspberry Pi Camera Module (preferred):
  sudo apt install python3-picamera2
"""

import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

parser = argparse.ArgumentParser(description='MJPEG camera server')
parser.add_argument('--port',   default=8081, type=int)
parser.add_argument('--width',  default=640,  type=int)
parser.add_argument('--height', default=480,  type=int)
parser.add_argument('--fps',    default=20,   type=int)
parser.add_argument('--source', default=0,    help='cv2 capture index or /dev/videoX')
args = parser.parse_args()

# ── Shared frame buffer ───────────────────────────────────────────────────────

_lock        = threading.Lock()
_frame_jpeg  = b''
_frame_count = 0


def capture_loop():
    global _frame_jpeg, _frame_count
    try:
        import cv2
    except ImportError:
        print("ERROR: opencv-python not installed. Run:  pip3 install opencv-python")
        raise

    src = int(args.source) if str(args.source).isdigit() else args.source
    cap = cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS,          args.fps)

    if not cap.isOpened():
        print(f"ERROR: Cannot open camera source: {src}")
        raise RuntimeError("Camera not available")

    print(f"[CAM] Opened {src} → {args.width}x{args.height} @ {args.fps} fps")

    interval = 1.0 / args.fps
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if ret:
            # Encode frame as JPEG
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with _lock:
                    _frame_jpeg  = buf.tobytes()
                    _frame_count += 1
        elapsed = time.time() - t0
        time.sleep(max(0, interval - elapsed))


# ── HTTP MJPEG server ─────────────────────────────────────────────────────────

BOUNDARY = b'mjpegboundary'

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # suppress default access log

    def do_GET(self):
        if self.path not in ('/', '/stream', '/video'):
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type',
                         f'multipart/x-mixed-replace; boundary={BOUNDARY.decode()}')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        try:
            while True:
                with _lock:
                    frame = _frame_jpeg

                if frame:
                    header = (
                        b'--' + BOUNDARY + b'\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(frame)).encode() + b'\r\n\r\n'
                    )
                    self.wfile.write(header + frame + b'\r\n')
                    self.wfile.flush()
                time.sleep(1.0 / args.fps)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client disconnected – normal


# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    # Start capture in background thread
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    # Give camera a moment to warm up
    time.sleep(1.0)

    addr = ('0.0.0.0', args.port)
    httpd = HTTPServer(addr, MJPEGHandler)
    print(f'[CAM] MJPEG stream: http://<drone-ip>:{args.port}/stream')
    print('[CAM] Press Ctrl+C to stop.')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n[CAM] Stopped.')
