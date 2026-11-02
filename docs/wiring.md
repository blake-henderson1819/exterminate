# Exterminate wiring

## Perfboard rails

| Row | Net |
|---|---|
| 1 | 5V |
| 3 | GND |
| 16 | VBAT (11–12.6 V, 3S LiPo) |
| 18 | GND |

Rows 3 and 18 are jumpered, so both GND rails are common.

## Connections

| Device | Connection |
|---|---|
| Battery (850 mAh 3S LiPo) | Screw terminal on rows 16 (+) and 18 (−), with a 25 V capacitor across it |
| Arduino Nano | 5V → row 1, GND → row 18, VIN → row 16 |
| HotRC F-06A receiver | CH1–CH3 → A1–A3, 5V → row 1, GND → row 3 |
| Weapon ESC (Readytosky S40) | Signal → D3 |
| Drive ESC headers (planned) | D5 (left), D6 (right) |
| Raspberry Pi 5 | Powered by a 5 V 5 A USB-C buck converter fed from rows 16 and 18; talks to the Nano over USB |

## Notes

- The current firmware (`firmware/ExterminateRC/ExterminateRC.ino`) drives a
  TB6612 on D4–D10 and uses `analogWrite` on D5/D6. To use the planned drive ESC
  headers on D5/D6, the firmware must be changed to send RC pulses
  (1000–2000 µs) on those pins first.
