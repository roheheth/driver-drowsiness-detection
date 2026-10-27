"""
Mock ESP32 Telemetry & Loopback Simulator
=========================================
Emulates the ESP32 hardware for testing and evaluating the AI Vision layer
on a PC when no physical ESP32 or sensors are plugged in.

Features:
- Prints real-time fused decision evaluations
- Simulates realistic resting/drowsy physiological drift (GSR slope + SDNN)
- Visualizes incoming serial commands from drowsiness_detection.py
"""

import time
import random

def run_mock_esp32():
    print("=================================================================")
    print("[MOCK ESP32] Virtual Hardware Simulator Running")
    print("[MOCK ESP32] Emulating AD8232 ECG + GSR v1.2 + MPU6050 IMU")
    print("[MOCK ESP32] Baud Rate: 115200 | Fused Loop: 5 Hz (200 ms)")
    print("=================================================================\n")

    baseline_sdnn = 35.0
    baseline_scl = 320.0
    current_scl = baseline_scl
    imu_stable = 1
    eye_closed = 0
    fatigue = 0

    try:
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            
            # Simulate slight physiological relaxation after 15 seconds
            if elapsed > 15:
                current_sdnn = baseline_sdnn + min(50.0, (elapsed - 15) * 1.5)
                current_scl = max(180.0, baseline_scl - (elapsed - 15) * 3.0)
                scl_slope = -0.55  # Exceeds -0.5 uS/s threshold
            else:
                current_sdnn = baseline_sdnn + random.uniform(-2, 3)
                scl_slope = random.uniform(-0.05, 0.05)

            parasympathetic_rise = current_sdnn > (baseline_sdnn + 45.0)
            sympathetic_drop = scl_slope < -0.5

            alert = (eye_closed == 1) or ((fatigue == 1 or parasympathetic_rise) and sympathetic_drop and (imu_stable == 1))

            status = "[ALERT] BUZZER ON " if alert else "[SAFE]  BUZZER OFF"
            print(f"{status} | EYE:{eye_closed} FAT:{fatigue} | SDNN:{current_sdnn:.1f}ms | SCL:{current_scl:.1f}uS (Slope:{scl_slope:.2f}) | IMU_OK:{imu_stable}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[MOCK ESP32] Simulator terminated cleanly.")

if __name__ == '__main__':
    run_mock_esp32()
