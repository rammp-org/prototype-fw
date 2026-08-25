# pear_platform

ESP-IDF firmware project for the Pear Platform prototype. It configures four
Reflex RMD-X6-S2 motor actuators with IDs 1 through 4.

## Development

This project uses ESP-IDF and the ESP++ `esp32-p4-eth` component. The default
target is ESP32-P4.

```console
idf.py build
idf.py -p PORT flash monitor
```
