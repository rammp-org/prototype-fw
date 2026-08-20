# five_bar_v1

ESP-IDF firmware project for the five-bar prototype, based on the original
`five_bar` application.

## Development

This project uses ESP-IDF and the ESP++ `esp32-p4-eth` component. The default
target is ESP32-P4.

```console
idf.py build
idf.py -p PORT flash monitor
```