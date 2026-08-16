# cvrobottest — skin-tone detection with serial camera control 🤖

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-2%2F3-blue.svg)](#)

Detect **human skin tone** in a camera feed and draw a boundary around it —
useful for gesture recognition and motion tracking. Drives a pan/tilt camera
over a serial link to keep the detected region centred.

Inspired by a [StackOverflow answer](http://stackoverflow.com/a/14756351/1463143).

## How it works

- Capture frames from the primary camera with OpenCV (`cv2.VideoCapture`).
- Convert to **YCrCb** colour space and threshold on the skin-tone range
  (`Y:0–255, Cr:133–173, Cb:77–127`).
- Find the detected region's bounding box and draw it.
- Send steering commands over serial (`/dev/ttyUSB0`, 9600 baud) to centre the
  region in view (`bw/bh`, `lbx/lby`, `rbx/rby` track the target box).

## Requirements

- Python, OpenCV (`cv2`), NumPy, `pyserial`

## Run

```bash
python cvrobottest.py
```

Adjust the serial port and camera index to match your hardware.

## License

[MIT](LICENSE) © Valentin Heinitz
