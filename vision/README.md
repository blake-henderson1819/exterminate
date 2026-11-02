# Exterminate: face-tracking drive test (bench prototype)

The Arduino Nano runs the TB6612FNG drive motors. A Raspberry Pi 5 **or a Mac**
watches a camera, finds the largest face, and tells the Nano over USB serial to
spin toward it.

```
face_track/
├── face_drive/face_drive.ino   # Nano firmware (drive only, weapon held disarmed)
├── face_track.py               # Pi / Mac face tracker
├── requirements.txt            # pip deps for the Mac (opencv-python, pyserial)
└── README.md
```

## ⚠️ Safety first

- **Remove the spinner** from the weapon motor before any test. The firmware holds
  the weapon ESC on D3 at 1000 us (disarmed) and never spins it, but don't rely on
  software alone.
- **Wheels off the ground** for the first runs. Prop the bot on a block so the
  wheels spin freely.
- **Unplug the LiPo** whenever you're not actively testing.
- Keep a hand near the LiPo connector. **Ctrl-C** in the terminal sends `S` (stop).
  If the Pi/Mac crashes or the cable comes out, the Nano stops the motors by
  itself within 300 ms.

## Wiring (as built)

| Function | Nano pin |
|---|---|
| TB6612 PWMA / AIN1 / AIN2 (left motor) | D5 / D7 / D8 |
| TB6612 PWMB / BIN1 / BIN2 (right motor) | D6 / D9 / D10 |
| TB6612 STBY | D4 |
| Weapon ESC signal | D3 (held at 1000 us) |
| HotRC receiver CH1-CH3 | A1-A3 (inputs, unused here) |

## 1. Upload the sketch (Arduino IDE on the Mac)

1. Open `face_drive/face_drive.ino` in the Arduino IDE. The folder name must match
   the sketch name, so keep the `face_drive/` folder.
2. **Tools → Board → Arduino AVR Boards → Arduino Nano**.
3. **Tools → Processor → ATmega328P**. If the upload fails with
   `avrdude: stk500_recv(): programmer is not responding` (common on CH340
   clones), switch to **ATmega328P (Old Bootloader)** and try again.
4. **Tools → Port**: pick `/dev/cu.usbserial-XXXX` (or `cu.wchusbserial…`).
   Recent macOS has a built-in CH340 driver. If no port appears, try another USB
   cable (some only carry power) or install the WCH CH34x driver.
5. Click **Upload**.
6. Open **Serial Monitor** at **115200** baud with **Newline** line ending. You
   should see `READY`, then `STAT L=0 R=0 WD=1` once a second. With the LiPo
   unplugged, you can type `M 100 100` and the reply should be `OK`. About 0.3 s
   later you get `TIMEOUT`, because the watchdog stops the motors when commands
   don't keep coming.
7. **Close the Serial Monitor** before running `face_track.py`. Only one program
   can open the port at a time.

## 2a. Run on the Mac (testing today)

```bash
cd exterminate_fw/face_track
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt          # opencv-python, pyserial
```

Picamera2 is not needed on the Mac. The script only tries it on Linux.

1. **Camera permission**: the first run will ask for camera access for your
   terminal app (Terminal / iTerm / VS Code). If it fails silently, enable it in
   *System Settings → Privacy & Security → Camera* and restart the terminal.
2. **Dry run (no serial, no motors):**
   ```bash
   python3 face_track.py --dry-run --show
   ```
   A window opens with a green box on your face. The terminal prints commands such
   as `M -110 110` and `S` as you move left and right. Press `q` in the window or
   Ctrl-C to quit.
3. **Pick the camera**: `--camera 0` is usually the built-in FaceTime camera.
   A Logitech webcam is usually `--camera 1`. Try both with `--dry-run --show`.
   (With an iPhone nearby, Continuity Camera may take an index too.)
4. **Real run, wheels off the ground, spinner removed:**
   ```bash
   python3 face_track.py --show                       # auto-finds /dev/cu.usbserial*
   python3 face_track.py --show --port /dev/cu.usbserial-1410 --camera 1
   ```
   The script waits for `READY` because the Nano resets when the port opens.

## 2b. Run on the Raspberry Pi 5 (Raspberry Pi OS Bookworm)

```bash
sudo apt update
sudo apt install -y python3-opencv python3-picamera2 python3-serial
```

pip is **not** needed on the Pi, because apt provides all three packages. Don't
pip-install opencv into the system Python on Bookworm.

Let your user open the serial port, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

Copy the `face_track/` folder to the Pi and plug the Nano into a Pi USB port. It
shows up as `/dev/ttyUSB0` (CH340) or `/dev/ttyACM0`.

1. **Dry run first:**
   ```bash
   python3 face_track.py --dry-run          # add --show if a desktop/monitor is attached
   ```
2. **Real run, wheels off the ground, spinner removed:**
   ```bash
   python3 face_track.py
   python3 face_track.py --port /dev/ttyUSB0 --speed-max 120
   ```

The Pi camera module (Picamera2) is used first. If it isn't present, the script
falls back to a USB webcam via OpenCV (`--camera N`).

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--port PATH` | auto | Serial port. Auto order: `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/cu.usbserial*`, `/dev/cu.wchusbserial*`, `/dev/cu.usbmodem*` |
| `--camera N` | 0 | OpenCV webcam index |
| `--dry-run` | off | Print commands, don't open serial |
| `--show` | off | Preview window with face box and deadband lines |
| `--speed-max N` | 160 | Max motor speed (0-255) |
| `--approach` | off | Creep forward when the face is centered and small (far away) |
| `--search` | off | Slowly spin when no face is seen (default: stop) |

## Serial protocol (115200 baud, newline-terminated)

| Pi → Nano | Meaning |
|---|---|
| `M <left> <right>` | Motor speeds -255..255, clamped. Positive = forward |
| `S` | Stop both motors |
| `P` | Ping, replies `PONG` |

| Nano → Pi | Meaning |
|---|---|
| `READY` | Boot complete, motors stopped |
| `OK` | Valid `M`/`S` received |
| `ERR <reason>` | Bad command. It doesn't feed the watchdog |
| `TIMEOUT` | No valid command for 300 ms, so motors stopped |
| `STAT L=<l> R=<r> WD=<0/1>` | Status, about 1 per second |

The tracker sends a command about 20 times per second. Spinning right is
`M +n -n`, and spinning left is `M -n +n`.

## Flipping motor directions

Test with the wheels off the ground using the Serial Monitor (LiPo connected,
spinner removed). Send `M 100 100` and resend it quickly, since the watchdog
stops the motors after 300 ms. Both wheels should turn **forward**.

- If one wheel turns backward, open `face_drive.ino` and set `INVERT_LEFT` or
  `INVERT_RIGHT` to `true`, then re-upload. (Swapping that motor's two wires on
  the TB6612 does the same thing.)
- If the wheels are correct but **left and right are swapped** (`M 100 0` turns
  the right wheel), swap the motor connectors between A and B on the TB6612.
- If both wheels are correct but the bot turns **away** from your face, set
  `TURN_SIGN = -1` in `face_track.py`.

## Tunable constants

**face_drive.ino**: `INVERT_LEFT`, `INVERT_RIGHT`, `CMD_TIMEOUT_MS = 300`,
`STATUS_EVERY_MS = 1000`, `SPEED_LIMIT = 255`, `WEAPON_SAFE_US = 1000`.

**face_track.py**:

| Constant | Default | Effect |
|---|---|---|
| `FRAME_W, FRAME_H` | 320×240 | Detection resolution (bigger = slower, finds smaller faces) |
| `SEND_HZ` | 20 | Command rate |
| `DEADBAND_FRAC` | 0.08 | Face within ±8% of frame width from center counts as centered, so the bot stops |
| `KP` | 1.0 | Proportional gain on the normalized horizontal error |
| `TURN_MIN` / `TURN_MAX` | 70 / 160 | Turn speed floor (beats friction) and default cap |
| `FORWARD_SPEED` | 80 | `--approach` forward speed |
| `FACE_SMALL_FRAC` | 0.25 | Face narrower than this fraction of the frame counts as far away |
| `SEARCH_SPEED`, `SEARCH_DIR` | 75, +1 | `--search` spin |
| `TURN_SIGN` | 1 | Flip if it turns away from the face |
| `HAAR_SCALE`, `HAAR_NEIGHBORS`, `HAAR_MIN_SIZE` | 1.1, 5, 30×30 | Detector sensitivity. Raise neighbors if you get false faces |

If the bot oscillates around the face, lower `KP` or `--speed-max`, or widen
`DEADBAND_FRAC`. If it stalls short of centering, raise `TURN_MIN`.
