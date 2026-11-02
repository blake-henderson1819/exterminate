#!/usr/bin/env python3
# Exterminate - step 1: smooth turn-only person tracking (no driving).
# Camera, detection, and motor control each run on their own, so a slow
# detection frame never leaves the motors spinning on old information.
import time, glob, threading, cv2, serial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

W, H = 640, 360
KP = 0          # turn power per unit of error (error runs -1..1)
KD = 0           # damping, resists overshoot
MIN_TURN = 190    # smallest power that actually moves the motors
MAX_TURN = 190
DEADBAND = 0.12   # this close to center counts as centered (stop)
SMOOTH = 0.0      # 0..1, higher = smoother but slower to react
STALE_S = 0.35    # no fresh detection for this long -> stop
CTRL_HZ = 30
MIN_ON_S = 0.08    # once a turn starts, keep motors on at least this long
REST_S = 0.30      # after stopping, wait this long before the next turn
LEAD_S = 0.5      # predict where you'll be this far ahead (camera lag + coasting)

# ---------- Nano serial ----------
SER = [None]
def connect():
    while True:
        ports = glob.glob('/dev/ttyAMA0') or glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if ports:
            try:
                SER[0] = serial.Serial(ports[0], 115200, timeout=0.05); time.sleep(2)
                print("connected", ports[0]); return
            except Exception as e: print("open failed", e)
        else: print("waiting for Nano...")
        time.sleep(1)
def write(b):
    try: SER[0].write(b); SER[0].reset_input_buffer()
    except Exception as e:
        print("Nano dropped:", e)
        try: SER[0].close()
        except Exception: pass
        connect()
connect()

# ---------- shared state ----------
LOCK = threading.Lock()
S = {"frame": None, "err": None, "t": 0.0, "boxes": [], "cmd": "S", "jpeg": None}

# ---------- camera thread: always holds the newest frame ----------
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (W, H), "format": "RGB888"},
              transform=Transform(hflip=1, vflip=1), buffer_count=2))
cam.start()
def cam_loop():
    while True:
        f = cam.capture_array()
        with LOCK: S["frame"] = f
threading.Thread(target=cam_loop, daemon=True).start()

# ---------- control thread: steady 30 Hz, smooth proportional turning ----------
def ctrl_loop():
    sm, prev, prev_t, last_t, d = 0.0, 0.0, time.time(), 0.0, 0.0
    on_dir, on_t, off_t = 0, 0.0, 0.0
    while True:
        now = time.time()
        if on_dir and now - on_t < MIN_ON_S:          # committed: keep turning
            p = on_dir * MAX_TURN; write(f"M {int(p)} {int(-p)}\n".encode())
            time.sleep(1 / CTRL_HZ); continue
        with LOCK: err, t = S["err"], S["t"]
        if err is None or now - t > STALE_S:
            if on_dir: off_t = now
            on_dir = 0
            sm = 0.0; prev = 0.0; last_t = 0.0; d = 0.0; cmd = "S"; write(b"S\n")
        else:
            sm = SMOOTH * sm + (1 - SMOOTH) * err
            if t != last_t:                 # only measure speed on a new detection
                if last_t: d = (sm - prev) / max(t - last_t, 1e-3)
                prev, last_t = sm, t
            pred = sm + d * LEAD_S          # where the error is heading
            if abs(sm) < DEADBAND or abs(pred) < DEADBAND or pred * sm < 0 or (not on_dir and now - off_t < REST_S):
                if on_dir: off_t = now
                on_dir = 0; cmd = "S"; write(b"S\n")
            else:
                p = KP * sm + KD * d
                mag = min(MAX_TURN, MIN_TURN + abs(p))
                p = mag if sm > 0 else -mag     # direction always from where you are
                nd = 1 if p > 0 else -1
                if nd != on_dir: on_dir, on_t = nd, now
                l, r = int(p), int(-p)
                cmd = f"M {l} {r}"; write(f"M {l} {r}\n".encode())
        prev_t = now
        with LOCK: S["cmd"] = cmd
        time.sleep(1 / CTRL_HZ)
threading.Thread(target=ctrl_loop, daemon=True).start()

# ---------- video stream (view only) ----------
class Hd(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f'); self.end_headers()
        while True:
            j = S["jpeg"]
            if j is not None:
                self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n\r\n' + j + b'\r\n')
            time.sleep(0.08)
threading.Thread(target=lambda: ThreadingHTTPServer(('', 8000), Hd).serve_forever(), daemon=True).start()

# ---------- detection loop (main thread) ----------
model = YOLO('yolov8n.pt')
print("turn-only tracking; Ctrl+C to stop")
fps, last = 0.0, time.time()
try:
    while True:
        with LOCK: frame = S["frame"]
        if frame is None: time.sleep(0.01); continue
        res = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)[0]
        boxes = [tuple(map(int, b)) for b in res.boxes.xyxy.tolist()]
        now = time.time()
        if boxes:
            a, b, c, d = max(boxes, key=lambda q: (q[2]-q[0]) * (q[3]-q[1]))
            with LOCK: S["err"], S["t"] = ((a + c) / 2 - W / 2) / (W / 2), now
        fps = 0.8 * fps + 0.2 / max(now - last, 1e-3); last = now
        vis = frame.copy()
        cv2.line(vis, (W//2, 0), (W//2, H), (255, 255, 0), 1)
        for (a, b, c, d) in boxes: cv2.rectangle(vis, (a, b), (c, d), (0, 255, 0), 2)
        cv2.putText(vis, f"{S['cmd']}  {fps:.1f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        S["jpeg"] = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes()
except KeyboardInterrupt:
    pass
finally:
    write(b"S\n"); cam.stop()
