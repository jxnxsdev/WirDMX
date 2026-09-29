# WirDMX

ESP32 (ESPNOW) based wireless DMX communication ecosystem

# What is this project?

This project includes programs for the ESP32 to turn them into components
of a wireless DMX network which communicated over ESP-NOW.

# Components

## DMX Sender

`WirDMX Sender` entrypoint into the Network. It turns Physical DMX into
network messages

### Required components:

- ESP32 (for example: WROOM-32)
- RS485 to UART module (for example: MAX485)
- 1x WS2812 LED (optional; used for status display)

### Connections:

- Connect 5V and GND to the ESP32, UART Module and the WS2812 LED
- Connect the RO (Receive out, sometimes called RX) onto pin 16 of the ESP32
- Connect the DIn of the WS2812 LED to pin 5 on the ESP32

### Settings:

You can define pins used, ESP-NOW Channel, etc. in the top of the main.cpp file

## DMX Receiver

`WirDMX Receiver` reads DMX values from the Network and turns them back into
physical DMX.

### Required components:

- ESP32 (for example WROOM-32)
- RS485 to UART module (for example: MAX485)
- 1x WS2812 LED (optional; used for status display)

### Connections:

- Connect 5V and GND to the ESP32, UART Module and the WS2812 LED
- Connect the DI (Data in, sometimes called TX) onto pin 17 of the ESP32
- Connect the DIn of the WS2812 LED to pin 5 on the ESP32

### Settings:

You can define pins used, ESP-NOW Channel, etc. in the top of the main.cpp file

# Status LED Meanings

- Red: No DMX Received
- blinking Green: DMX is Received
- Orange: Corrupted data received
