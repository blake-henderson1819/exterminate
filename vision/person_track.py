#!/usr/bin/env python3
# Exterminate - whole-person tracker (OpenCV HOG people detector).
# Turns toward the biggest person and drives toward them; backs off if too close.
import time, glob, cv2, serial, numpy as np, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
JPEG=[None]; CMD=['S']
class H(BaseHTTPRequestHandler):
    def log_message(self,*a): pass
    def do_GET(self):
        self.send_response(200); self.send_header('Content-Type','multipart/x-mixed-replace; boundary=f'); self.end_headers()
        while True:
            j=JPEG[0]
            if j is not None:
                self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n\r\n'+j+b'\r\n')
            time.sleep(0.08)
threading.Thread(target=lambda: ThreadingHTTPServer(('',8000),H).serve_forever(),daemon=True).start()
from picamera2 import Picamera2
from libcamera import Transform

W, H = 640, 360
MIN_TURN = 130      # raised for weak motors
MAX_SPEED = 255
FWD_SPEED = 180
TURN_GAIN = 300     # per unit of normalized x error
CENTER_BAND = 0.15  # fraction of width where we drive straight
NEAR_FRAC = 0.80    # person box taller than this fraction of frame -> stop
HOLD_S = 0.5
TURN_SPEED = 200  # power for each nudge
NUDGE_S = 0.08   # how long each nudge lasts (shorter = smaller turns)
SETTLE_S = 0.25   # pause after a nudge so the camera catches up
SM = [0.0]

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
    try:
        SER[0].write(b); SER[0].reset_input_buffer()
    except Exception as e:
        print("Nano dropped:", e)
        try: SER[0].close()
        except Exception: pass
        connect()
connect()
def send(l, r):
    l = int(max(-MAX_SPEED, min(MAX_SPEED, l))); r = int(max(-MAX_SPEED, min(MAX_SPEED, r)))
    CMD[0]=f"M {l} {r}"; write(f"M {l} {r}\n".encode())

from ultralytics import YOLO
model = YOLO('yolov8n.pt')   # downloads once
cam = Picamera2(); cam.configure(cam.create_preview_configuration(main={"size": (W, H), "format": "RGB888"}, transform=Transform(hflip=1, vflip=1))); cam.start()
last_seen, last_box = 0, None
print("tracking people")
try:
    while True:
        cam.capture_array()            # throw away a possibly stale buffered frame
        frame = cam.capture_array()
        res = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)[0]
        rects = [(int(a), int(b), int(c - a), int(d - b)) for a, b, c, d in res.boxes.xyxy.tolist()]
        now = time.time()
        if len(rects):
            i = int(np.argmax([w * h for (_, _, w, h) in rects]))
            last_box, last_seen = rects[i], now
        if last_box is not None and now - last_seen < HOLD_S:
            x, y, w, h = last_box
            err = ((x + w / 2) - W / 2) / (W / 2)       # -1..1, + = person right
            if abs(err) > CENTER_BAND:
                # small nudge toward the person, then stop and look again
                d = 1 if err > 0 else -1
                send(d * TURN_SPEED, -d * TURN_SPEED)
                time.sleep(NUDGE_S)
                send(0, 0)
                time.sleep(SETTLE_S)
                send(0, 0)
                last_seen = 0                          # need a fresh look before moving again
            else:
                send(FWD_SPEED, FWD_SPEED)
        else:
            CMD[0]="S"; write(b"S\n")
        vis=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR) if False else frame.copy()
        cv2.line(vis,(W//2,0),(W//2,H),(255,255,0),1)
        for (a,b,c,d) in rects: cv2.rectangle(vis,(a,b),(a+c,b+d),(0,0,255),1)
        if last_box is not None and now-last_seen<HOLD_S:
            a,b,c,d=last_box; cv2.rectangle(vis,(a,b),(a+c,b+d),(0,255,0),3)
        cv2.putText(vis,CMD[0],(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        JPEG[0]=cv2.imencode(".jpg",vis,[cv2.IMWRITE_JPEG_QUALITY,60])[1].tobytes()
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    write(b"S\n"); cam.stop()
