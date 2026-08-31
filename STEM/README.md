# STEM

ESP-IDF firmware project for the STEM prototype, based on the original
five-bar application.

## Development

This project uses ESP-IDF and the ESP++ `esp32-p4-eth` component. The default
target is ESP32-P4.

## Motor

- Motor: Reflex RMD-X6-S2
- Gear ratio: 1:36

```console
idf.py build
idf.py -p PORT flash monitor
```