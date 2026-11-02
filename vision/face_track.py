#!/usr/bin/env python3
"""
Exterminate - face_track.py

Raspberry Pi 5 side of the face-tracking bench test. Grabs camera frames,
finds the largest face with an OpenCV Haar cascade, and tells the Arduino
Nano (running face_drive.ino) to spin in place toward it.

Serial protocol (115200 baud, newline-terminated):
    M <left> <right>   motor speeds -255..255
    S                  stop
The Nano replies READY at boot, OK per command, and stops on its own if it
hears nothing for 300 ms.

Runs on Raspberry Pi OS Bookworm (Pi camera via Picamera2, or USB webcam)
and on macOS (built-in / Logitech webcam via OpenCV; Nano on /dev/cu.*).

Usage:
    python3 face_track.py --dry-run          # no serial, just print commands
    python3 face_track.py --show             # real run with preview window
    python3 face_track.py --port /dev/ttyUSB0 --speed-max 120
    python3 face_track.py --camera 1          # Mac: pick the Logitech webcam
"""

import argparse
import glob
import os
import platform
import sys
import time

import cv2

IS_MAC = platform.system() == "Darwin"

# ----------------------------- Tunables ------------------------------------
FRAME_W, FRAME_H = 320, 240   # small frames keep Haar detection fast
SEND_HZ = 20.0                # command rate to the Nano
DEADBAND_FRAC = 0.08          # |error| < 8% of frame width => centered
KP = 1.0                      # turn speed per unit normalized error (x speed_max)
TURN_MIN = 100                # smallest turn that actually overcomes friction
FACE_HOLD_S = 0.5             # keep using the last face this long after the detector drops it
TURN_MAX = 160                # default cap on turn speed (--speed-max overrides)
FORWARD_SPEED = 80            # forward speed when --approach and face centered
FACE_SMALL_FRAC = 0.25        # "small" face: width < 25% of frame width
SEARCH_SPEED = 75             # spin speed for --search when no face
SEARCH_DIR = 1                # +1 spin right, -1 spin left while searching
# Flip the sign if the bot turns AWAY from the face (camera mounted mirrored,
# or motor wiring swapped). Prefer fixing motor direction in the sketch first.
TURN_SIGN = 1
BAUD = 115200
READY_TIMEOUT_S = 5.0

# Haar detector settings
HAAR_SCALE = 1.1
HAAR_NEIGHBORS = 5
HAAR_MIN_SIZE = (30, 30)


# ----------------------------- Camera --------------------------------------
class Camera:
    """Picamera2 first (Pi camera module, Linux only), then OpenCV VideoCapture.

    Picamera2 is imported lazily so the script runs on a Mac without it.
    """

    def __init__(self, width, height, index=0):
        self.picam = None
        self.cap = None
        if not IS_MAC and self._try_picamera2(width, height):
            return
        # macOS: use the AVFoundation backend explicitly; elsewhere let OpenCV pick.
        if IS_MAC:
            self.cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
        else:
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            hint = (" On macOS, allow camera access for your terminal app in"
                    " System Settings > Privacy & Security > Camera." if IS_MAC else "")
            raise RuntimeError(f"No camera at index {index}.{hint}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        print(f"[camera] using OpenCV VideoCapture({index})")

    def _try_picamera2(self, width, height):
        try:
            from picamera2 import Picamera2  # type: ignore  (lazy, optional)
            self.picam = Picamera2()
            cfg = self.picam.create_preview_configuration(
                main={"size": (width, height), "format": "RGB888"})
            self.picam.configure(cfg)
            self.picam.start()
            time.sleep(0.5)  # let exposure settle
            print("[camera] using Picamera2")
            return True
        except Exception as e:  # no module, no camera, or camera busy
            print(f"[camera] Picamera2 unavailable ({e}); trying OpenCV webcam")
            self.picam = None
            return False

    def read(self):
        """Return a BGR frame or None."""
        if self.picam is not None:
            # Picamera2 "RGB888" is actually BGR byte order in memory, which
            # is what OpenCV wants, so no conversion is needed.
            return self.picam.capture_array()
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        # Webcams may ignore the size request (Mac cameras often give 1280x720);
        # downscale so detection stays fast and control math sees a small frame.
        if frame.shape[1] > FRAME_W * 1.5:
            scale = FRAME_W / float(frame.shape[1])
            frame = cv2.resize(frame, None, fx=scale, fy=scale)
        return frame

    def close(self):
        try:
            if self.picam is not None:
                self.picam.stop()
                self.picam.close()
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


# ----------------------------- Face detection ------------------------------
def load_cascade():
    name = "haarcascade_frontalface_default.xml"
    candidates = []
    data = getattr(cv2, "data", None)
    if data is not None and getattr(data, "haarcascades", None):
        candidates.append(os.path.join(data.haarcascades, name))
    candidates += [
        "/usr/share/opencv4/haarcascades/" + name,   # Debian/Bookworm apt
        "/opt/homebrew/share/opencv4/haarcascades/" + name,  # Homebrew (Apple Silicon)
        "/usr/share/opencv/haarcascades/" + name,
        "/usr/local/share/opencv4/haarcascades/" + name,
    ]
    for path in candidates:
        if os.path.isfile(path):
            cascade = cv2.CascadeClassifier(path)
            if not cascade.empty():
                print(f"[faces] cascade: {path}")
                return cascade
    raise RuntimeError("Haar cascade not found; tried:\n  " + "\n  ".join(candidates))


def largest_face(cascade, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(
        gray, scaleFactor=HAAR_SCALE, minNeighbors=HAAR_NEIGHBORS,
        minSize=HAAR_MIN_SIZE)
    if len(faces) == 0:
        return None
    return max(faces, key=lambda f: f[2] * f[3])  # (x, y, w, h)


# ----------------------------- Control -------------------------------------
def compute_command(face, frame_w, args):
    """Return (left, right) motor speeds for this frame."""
    if face is None:
        if args.search:
            s = SEARCH_SPEED * SEARCH_DIR
            return s, -s
        return 0, 0

    x, y, w, h = face
    err = ((x + w / 2.0) - frame_w / 2.0) / (frame_w / 2.0)  # -1..1, + = right

    if abs(err) < DEADBAND_FRAC * 2:  # deadband: 8% of width = 0.16 normalized
        # Centered. Optionally creep forward if the face looks far away.
        if args.approach and w < FACE_SMALL_FRAC * frame_w:
            f = min(FORWARD_SPEED, args.speed_max)
            return f, f
        return 0, 0

    turn = KP * abs(err) * args.speed_max
    turn = max(TURN_MIN, min(args.speed_max, turn))
    turn = int(round(turn)) * (1 if err > 0 else -1) * TURN_SIGN
    # Spin in place: positive turn = rotate right (left fwd, right back).
    return turn, -turn


# ----------------------------- Serial link ---------------------------------
def find_port():
    """First match of: Linux CH340 / native USB, then macOS CH340 / native USB.

    On macOS use the /dev/cu.* device (not /dev/tty.*), which does not block
    waiting for carrier detect. CH340 clones show up as cu.usbserial-XXXX or
    cu.wchusbserialXXXX depending on driver.
    """
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*",
                "/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/cu.usbmodem*"]
    for pat in patterns:
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return None


class NanoLink:
    def __init__(self, port):
        import serial  # python3-serial
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        print(f"[serial] opened {port}; waiting for READY (Nano resets on open)")
        deadline = time.time() + READY_TIMEOUT_S
        buf = b""
        while time.time() < deadline:
            buf += self.ser.read(64)
            if b"READY" in buf:
                print("[serial] Nano READY")
                self.ser.reset_input_buffer()
                return
        raise RuntimeError(f"No READY from Nano on {port} within {READY_TIMEOUT_S}s")

    def send(self, line):
        self.ser.write((line + "\n").encode("ascii"))
        # Drain replies (OK / STAT / TIMEOUT) so the buffer never fills.
        if self.ser.in_waiting:
            for reply in self.ser.read(self.ser.in_waiting).decode(errors="ignore").split():
                if reply.startswith("ERR") or reply == "TIMEOUT":
                    print(f"[nano] {reply}")

    def close(self):
        try:
            self.ser.write(b"S\n")
            self.ser.flush()
            time.sleep(0.05)
        finally:
            self.ser.close()


class DryLink:
    def send(self, line):
        print(f"[dry-run] {line}")

    def close(self):
        print("[dry-run] S")


# ----------------------------- Main ----------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Exterminate face tracker")
    p.add_argument("--port", help="serial port (default: auto /dev/ttyUSB*, /dev/ttyACM*,"
                                  " /dev/cu.usbserial*, /dev/cu.usbmodem*)")
    p.add_argument("--camera", type=int, default=0,
                   help="OpenCV camera index for USB/built-in webcams (default 0)")
    p.add_argument("--dry-run", action="store_true", help="print commands, no serial")
    p.add_argument("--show", action="store_true", help="preview window (needs a display)")
    p.add_argument("--speed-max", type=int, default=TURN_MAX,
                   help=f"max motor speed 0-255 (default {TURN_MAX})")
    p.add_argument("--approach", action="store_true",
                   help="drive forward slowly when face is centered and small (default off)")
    p.add_argument("--search", action="store_true",
                   help="slowly spin when no face is seen (default: stop)")
    a = p.parse_args()
    a.speed_max = max(0, min(255, a.speed_max))
    return a


def main():
    args = parse_args()

    show = args.show
    has_display = IS_MAC or os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    if show and not has_display:
        print("[show] no display found; running headless")
        show = False

    cascade = load_cascade()
    cam = Camera(FRAME_W, FRAME_H, args.camera)

    if args.dry_run:
        link = DryLink()
    else:
        port = args.port or find_port()
        if not port:
            cam.close()
            sys.exit("No serial port found (/dev/ttyUSB*, /dev/ttyACM*, /dev/cu.usbserial*,"
                     " /dev/cu.usbmodem*). Use --port or --dry-run.")
        link = NanoLink(port)

    period = 1.0 / SEND_HZ
    last_cmd = None
    held_face, held_t = None, 0.0
    try:
        while True:
            t0 = time.time()
            frame = cam.read()
            if frame is None:
                left, right = 0, 0
                face = None
            else:
                face = largest_face(cascade, frame)
                if face is not None:
                    held_face, held_t = face, t0
                elif held_face is not None and t0 - held_t < FACE_HOLD_S:
                    face = held_face  # bridge brief detector dropouts (stops twitching)
                left, right = compute_command(face, frame.shape[1], args)

            cmd = "S" if (left == 0 and right == 0) else f"M {left} {right}"
            # Always send (keeps the Nano watchdog fed); in dry-run only print changes.
            if not args.dry_run or cmd != last_cmd:
                link.send(cmd)
            last_cmd = cmd

            if show and frame is not None:
                h, w = frame.shape[:2]
                db = int(DEADBAND_FRAC * w)
                cv2.line(frame, (w // 2 - db, 0), (w // 2 - db, h), (80, 80, 80), 1)
                cv2.line(frame, (w // 2 + db, 0), (w // 2 + db, h), (80, 80, 80), 1)
                if face is not None:
                    x, y, fw, fh = face
                    cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                cv2.putText(frame, cmd, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("Exterminate face_track", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[main] Ctrl-C, stopping")
    finally:
        link.close()   # sends S
        cam.close()
        if show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
