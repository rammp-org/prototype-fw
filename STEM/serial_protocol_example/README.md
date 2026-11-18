# STEM serial protocol example

This is a separate ESP-IDF firmware project. It does not modify the existing
`STEM` firmware and does not use Ethernet or `espp::Cli`.

The firmware talks to the browser through UART0. Use a 3.3 V USB-to-UART
adapter connected to the board's UART0 TX, RX, and GND pins. The browser sees
the adapter as a serial port at `115200` baud, 8 data bits, no parity, and 1
stop bit (8N1). Do not connect the browser directly to the board's USB
Serial/JTAG port for this example.

## Build and flash

```console
cd STEM/serial_protocol_example
idf.py build
idf.py -p PORT flash monitor
```

The project reuses the existing `STEM/main/ik_5bar.cpp` implementation through
CMake. The current STEM sources are not edited.

## Web client

Serve the matching page from its directory:

```console
cd STEM/serial_protocol_example/web
python3 -m http.server 8000
```

Open <http://localhost:8000> in Chrome or Edge, connect the USB-UART adapter,
and select its serial port.

## Protocol

Commands are ASCII, newline-terminated, and case-sensitive:

```text
PING
HOME
MOVE <x_rel> <y_rel>
SET <right|left|seat> <degrees>
STATUS
STOP
RELEASE
```

Responses begin with `OK` or `ERROR`. `STATUS` returns virtual actuator
positions as a compact JSON object. Firmware validates numeric input and
executes one command at a time in the serial task. It also sends an unsolicited
`HEARTBEAT` line every second so the host can verify that UART0 receive works.
