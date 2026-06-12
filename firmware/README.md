# VeloGuard Firmware

## Files
`src/main_2phase_test.cpp` — simplified FSM for bench testing
`src/main_4phase_production.cpp` — full production algorithm

## Dependencies
ESPAsyncWebServer
AsyncTCP
Adafruit MPU6050
Adafruit Unified Sensor

## Hardware
Board: Waveshare ESP32-S3 SIM7670G
IMU: MPU6050 (SDA=GPIO15, SCL=GPIO16)
Modem power: GPIO33

## Flashing
Open in Arduino IDE, select ESP32S3 Dev Module, upload.
