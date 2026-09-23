# STEM Web Serial example

This is a host-side control page for the existing STEM firmware. It does not
modify the ESP-IDF project and does not use Ethernet, WebUSB, or a new firmware
protocol. The page sends the existing newline-delimited CLI commands over the
board's USB Serial/JTAG console.

## Run

From this directory, start a local server:

```console
python3 -m http.server 8000
```

Open <http://localhost:8000> in Chrome or Edge, click **Connect**, and select
the STEM USB serial device. The current firmware uses `115200` baud for the
console. Click **Home** or use the position pad to send `home` and `move x y`.

## Current commands

The page uses commands already registered by `STEM/main/main.cpp`:

- `move <x_rel> <y_rel>`
- `home`
- `set <right|left|seat> <degrees>`
- `release`
- `status`

There is currently no firmware `stop` command, so this example does not expose
a stop button. The browser UI does not replace firmware-side IK or actuator
limit checks.
