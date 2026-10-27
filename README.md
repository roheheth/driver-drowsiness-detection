# Non-Intrusive Multi-Modal Steering-Wheel Interface for Physiological Driver Drowsiness Detection

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Hardware: ESP32](https://img.shields.io/badge/Hardware-ESP32%20Xtensa%20LX6-red.svg)](https://www.espressif.com/)
[![Vision: MediaPipe](https://img.shields.io/badge/Vision-Google%20MediaPipe%20Face%20Mesh-green.svg)](https://developers.google.com/mediapipe)
[![Framework: OpenCV](https://img.shields.io/badge/Framework-OpenCV%20Python-orange.svg)](https://opencv.org/)
[![TRL: 3](https://img.shields.io/badge/Readiness%20Level-TRL%203%20Bench%20Validation-brightgreen.svg)]()

> **Project Review-3 / Research Paper Implementation**  
> School of Computer Science and Engineering (SCOPE), Vellore Institute of Technology (VIT), Vellore, India.

---

## 👥 Authors & Academic Credentials

- **Rohith Elindram** — Reg. No: `23BCT0100` (`rohith.elindram@vitstudent.ac.in`)
- **Ashwin Kumar.S** — Reg. No: `23BCT0084` (`ashwinkumar.s2023a@vitstudent.ac.in`)
- **Dr. Syamasudha Veeragandham** — Faculty Guide & Corresponding Author (`syamasudha.v@vit.ac.in`)

---

## 📌 Abstract & Overview

Driver drowsiness is a major contributor to global road-traffic fatalities. Conventional commercial Advanced Driver Assistance Systems (ADAS) rely on camera-based eyelid closure and head-pose monitoring, which are inherently reactive (alerting only 1–3 minutes before microsleep onset) and prone to lighting failures, facial occlusions, and consumer privacy concerns.

This project introduces a **non-intrusive, multi-modal steering-wheel interface** that continuously assesses the human Autonomic Nervous System (ANS) directly through the driver's palms. By measuring **Galvanic Skin Response (GSR)** and **single-lead Electrocardiogram (ECG)** via dry-contact electrodes embedded in the steering wheel rim, the system tracks sympathetic withdrawal and parasympathetic expansion on an ultra-low-power **ESP32 microcontroller**.

The edge pipeline is complemented by an auxiliary **AI Vision Layer** executing on a companion processor using **Google MediaPipe Face Mesh**, extracting real-time Eye Aspect Ratio (EAR) at 30 fps to provide a fast-path zero-latency safety interlock.

---

## 🏛️ System Architecture

```
                    +------------------------------------+
                    |        DRIVER'S PALMAR CONTACT      |
                    |    (Embedded Steering Wheel Grip)  |
                    +-----------------+------------------+
                                      |
            +-------------------------+-------------------------+
            |                                                   |
            v                                                   v
   +------------------+                                +------------------+
   |  AD8232 ECG AFE  |                                |  GSR Skin Sensor |
   | (Lead-I Cardiac) |                                | (Palmar Res.)    |
   +--------+---------+                                +--------+---------+
            | (Analog GPIO 34)                                  | (Analog GPIO 35)
            +-------------------------+-------------------------+
                                      |
                                      v
                    +------------------------------------+
                    |       ESP32 EDGE MICROCONTROLLER   |
                    |  - 12-bit ADC (250 Hz ECG, 50Hz GSR)|
                    |  - Pan-Tompkins R-Peak / SDNN calc |
                    |  - Finite-Difference GSR SCL Slope |
                    |  - MPU6050 IMU Motion Artefact Gate|
                    +-----------------+------------------+
                                      ^
                                      | USB Serial (115200 baud, 10 Hz)
                                      | "EYE:X,FAT:Y
"
                    +-----------------+------------------+
                    |    COMPANION LAPTOP AI VISION      |
                    |  - Web-Camera Capture (30 FPS)     |
                    |  - MediaPipe 468-pt Face Mesh      |
                    |  - Eye Aspect Ratio (EAR) Metric   |
                    |  - Blink Rate & Prolonged Closure  |
                    +-----------------+------------------+
                                      |
                                      v
                    +------------------------------------+
                    |        FUSED DECISION ENGINE       |
                    |      (Deterministic State Machine) |
                    +-----------------+------------------+
                                      |
                                      v
                    +------------------------------------+
                    |     ACTIVE AUDIBLE BUZZER / ECU    |
                    |        ALARM INTERLOCK (GPIO 25)   |
                    +------------------------------------+
```

---

## 🧮 Mathematical Formulations

### 1. Heart Rate Variability (SDNN)
Computed over a rolling circular buffer of $N$ valid normal-to-normal (NN) intervals (300 ms $\le 	ext{NN}_i \le$ 1500 ms):
$$	ext{SDNN} = \sqrt{rac{1}{N} \sum_{i=1}^{N} (	ext{NN}_i - \overline{	ext{NN}})^2}$$
An expansion of $	ext{SDNN} > 	ext{baseline} + lpha$ ($lpha = 45	ext{ ms}$) signifies increasing parasympathetic rest dominance.

### 2. Galvanic Skin Conductance Level (SCL) & Slope
Skin Conductance Level smoothed over moving window $W$:
$$	ext{SCL}(t) = rac{1}{W} \sum_{i=0}^{W-1} 	ext{GSR}(t - i)$$
The rate of autonomic arousal degradation is quantified by the finite-difference slope:
$$rac{\Delta\sigma}{\Delta t} = rac{	ext{SCL}(t) - 	ext{SCL}(t - T)}{T}$$
A persistent negative slope ($rac{\Delta\sigma}{\Delta t} < -eta$, with $eta = 0.5\ \mu	ext{S/s}$) indicates systemic eccrine sweat gland cessation (sympathetic withdrawal).

### 3. Eye Aspect Ratio (EAR)
Extracted from 6 periocular 3D landmarks ($P_1 \dots P_6$) per eye via MediaPipe:
$$	ext{EAR} = rac{\|P_2 - P_6\| + \|P_3 - P_5\|}{2 \cdot \|P_1 - P_4\|}$$
$$	ext{EAR}_{	ext{avg}} = rac{	ext{EAR}_{	ext{left}} + 	ext{EAR}_{	ext{right}}}{2}$$
- **Threshold**: $	heta_{	ext{EAR}} = 0.22$.
- **Fast-path closure**: Asserted when $	ext{EAR}_{	ext{avg}} < 0.22$ persists for $\ge 10$ consecutive frames ($pprox 333	ext{ ms}$).

### 4. Unified Fused Decision Rule
$$	ext{ALERT} = [	ext{EYE\_CLOSED} = 1] \lor \left( [	ext{FATIGUE} = 1] \land \left[rac{\Delta\sigma}{\Delta t} < -eta
ight] \land [	ext{IMU\_stable} = 1] 
ight)$$
- **Condition 1 (Fast-Path Interlock)**: Sustained eye closure triggers the alarm immediately ($pprox 333	ext{–}400	ext{ ms}$ total latency) irrespective of the physiological state.
- **Condition 2 (Secondary Cross-Validated Path)**: Fatigue flagged by elevated blink rate ($\ge 25	ext{ bpm}$) is escalated to an alert **only** if corroborated by sympathetic skin conductance degradation and undisturbed vehicle kinematics.

---

## 🔌 Hardware Pinout & Wiring

| Component | Pin | ESP32 GPIO | Description |
| :--- | :--- | :--- | :--- |
| **GSR Sensor v1.2** | Analog OUT | **GPIO 35** | Electrodermal skin conductance |
| **AD8232 ECG** | OUTPUT | **GPIO 34** | Lead-I cardiac potential |
| **MPU6050 IMU** | SDA / SCL | **GPIO 21 / 22** | I2C motion artifact rejection |
| **Active Buzzer** | Signal (+) | **GPIO 25** | Audible alarm output |
| **Serial Link** | USB (D+/D-) | **UART0** | 115200 baud bidirectional link |

---

## 🚀 Setup & Execution Guide

### 1. ESP32 Firmware Upload
1. Open the Arduino IDE.
2. Install the ESP32 Board package via Board Manager.
3. Install required libraries:
   - `Wire` (built-in)
   - `MPU6050` by Electronic Cats / Jeff Rowberg
4. Open `esp32_firmware/esp32_drowsiness_complete/esp32_drowsiness_complete.ino`.
5. Select board **ESP32 Dev Module**, select your COM port, and click **Upload**.

### 2. Laptop AI Vision Layer Setup
1. Ensure Python 3.8+ is installed.
2. Navigate to `laptop_ai_vision/` and install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Run the vision inference script:
   ```bash
   python drowsiness_detection.py
   ```
4. The system will auto-detect the connected ESP32 on the USB COM port, display the real-time HUD with facial landmarks, EAR telemetry, and live GSR conductance level bar. Press `q` to safely exit.

### 3. Running Automated Tests & Hardware-Free Simulation
- **Automated Mathematical Unit Tests:**  
  Run the automated validation suite covering EAR, SDNN, GSR slope, and decision logic:
  ```bash
  python tests/test_pipeline.py
  ```
- **Virtual ESP32 Hardware Simulator:**  
  To test the vision layer or review telemetry without an ESP32 connected:
  ```bash
  python laptop_ai_vision/mock_esp32_simulation.py
  ```

---

## 📁 Repository Directory Structure

```
driver-drowsiness-detection/
├── .gitignore                                           # Standard Git ignore configurations
├── LICENSE                                              # MIT License
├── README.md                                            # Comprehensive project documentation
├── docs/
│   └── Research_Paper_23BCT0100_23BCT0084.pdf           # Full research paper
├── esp32_firmware/
│   ├── esp32_drowsiness_complete/
│   │   └── esp32_drowsiness_complete.ino                # Full multimodal firmware (ECG SDNN, GSR slope, IMU, buzzer)
│   └── esp32_drowsiness_fastpath/
│       └── esp32_drowsiness_fastpath.ino                # Fast-path serial & threshold interlock firmware
├── hardware/
│   ├── PINOUT.md                                        # Pinout table, wiring, and sensor specifications
│   ├── circuit_diagram.png                              # Wiring schematic diagram
│   ├── bench_prototype.png                              # Physical breadboard prototype photo
│   └── system_architecture.png                          # System architecture block diagram
├── laptop_ai_vision/
│   ├── drowsiness_detection.py                          # MediaPipe Face Mesh EAR & blink rate vision engine
│   ├── mock_esp32_simulation.py                         # Virtual hardware simulator for desktop testing
│   └── requirements.txt                                 # Python dependencies (OpenCV, MediaPipe, PySerial)
└── tests/
    └── test_pipeline.py                                 # Automated algorithmic unit tests
```

---

## 📊 Bench Validation Results (TRL 3)

| Phase | Evaluation Objective | Result Obtained |
| :---: | :--- | :--- |
| **1** | Hardware assembly & ECG/GSR signal verification | Clear PQRST cardiac wave observed; resting GSR 160–420 µS |
| **2** | Embedded Pan-Tompkins R-peak detection & SDNN | >90% visual concordance; NN intervals 620–980 ms; SDNN 22–55 ms |
| **3** | GSR Moving Average & SCL Trend | Measurable downward SCL drift and negative slope under seated relaxation |
| **4** | Integrated Edge Firmware End-to-End Alert | SDNN expansion (+45 ms) and negative slope (-0.5 µS/s) triggered buzzer reliably across 3 trials |
| **5** | AI Vision Serial Integration & Fusion | Fast-path EYE_CLOSED detected within 333 ms; FATIGUE cross-validated by low GSR (<1800 ADC) across 5 trials |

---

## 📄 License
This project is licensed under the [MIT License](LICENSE).
