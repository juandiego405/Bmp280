# Bmp280
Repository contains C code for ESP32 configuring BMP280 sensor without libraries via I2C connection.

**Wiring Instructions:*
Proper wiring is crucial for the accurate functioning and communication between the sensor and the ESP32. Follow these guidelines for optimal results:

- Ensure SD0 is connected to ground for a memory address of 0x76, or to VCC for an address of 0x77.
- Connect SDA (Data) to Pin 21 and SCL (Clock) to Pin 22 on the ESP32.
