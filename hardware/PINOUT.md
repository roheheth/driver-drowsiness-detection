# Hardware Pinout & Wiring Guide

## Pin Connections (ESP32 Dev Module)

| Component | Pin / Signal | ESP32 GPIO | Description / Operating Range |
| :--- | :--- | :--- | :--- |
| **GSR Sensor v1.2** | Signal (A0) | **GPIO 35** | 12-bit ADC input; detects skin conductance changes (alert: 250-450 µS, drowsy: 128-250 µS) |
| **GSR Sensor v1.2** | VCC / GND | 3.3V / GND | Regulated power supply |
| **AD8232 ECG Module** | OUTPUT | **GPIO 34** | Analog Lead-I cardiac potential input sampled at 250 Hz |
| **AD8232 ECG Module** | 3.3V / GND | 3.3V / GND | Power connections (CMRR > 80 dB) |
| **MPU6050 6-DOF IMU** | SDA | **GPIO 21** (Default I2C) | I2C Data line for angular velocity / motion artifact gating |
| **MPU6050 6-DOF IMU** | SCL | **GPIO 22** (Default I2C) | I2C Clock line |
| **MPU6050 6-DOF IMU** | VCC / GND | 3.3V / GND | Sensor power |
| **Active Buzzer** | Positive (+) | **GPIO 25** | Digital output (HIGH = Alarm ON, LOW = Alarm OFF) |
| **Active Buzzer** | Negative (-) | GND | Ground return |
| **Laptop AI Vision Link** | Micro-USB | UART0 (GPIO 1/3) | 115200 baud serial telemetry receiving `"EYE:X,FAT:Y\n"` at 10 Hz |

## Motion Artifact Rejection Gate
- The MPU6050 gyroscope monitors rotational rates across steering movements.
- If $|\omega_z| > 40^\circ/\text{s}$, steering motions/hand adjustments are detected; physiological buffer updates are momentarily suspended to prevent false alerts.
