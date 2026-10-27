/*
 ================================================================================================
 Non-Intrusive Multi-Modal Steering-Wheel Interface for Physiological Driver Drowsiness Detection
 ================================================================================================
 Board         : ESP32 Dev Module (Xtensa Dual-Core 240 MHz, 12-bit ADC)
 Biomarkers    : 
   - AD8232 ECG AFE (GPIO 34)   -> Sampled @ 250 Hz, Pan-Tompkins R-peak, Rolling SDNN (HRV)
   - GSR v1.2 Sensor (GPIO 35)  -> Sampled @ 50 Hz, 3s Moving Average SCL, Finite-Difference Slope
   - MPU6050 6-DOF IMU (I2C)    -> Gyro Z-axis angular velocity motion artifact gating (|w_z| <= 40 deg/s)
   - Companion AI Vision Link   -> UART 115200 baud, 10 Hz "EYE:X,FAT:Y\n" ASCII protocol
 Alert Actuator: Active Buzzer (GPIO 25)
 
 Authors       : Rohith Elindram (23BCT0100), Ashwin Kumar.S (23BCT0084)
 Guide         : Dr. Syamasudha Veeragandham
 Institution   : Vellore Institute of Technology (VIT), Vellore
 ================================================================================================
*/

#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// ─── Hardware Pin Configuration ───────────────────────────────────────────────────────────────
#define PIN_ECG_IN      34   // ADC1_CH6: Analog output from AD8232 ECG instrumentation amplifier
#define PIN_GSR_IN      35   // ADC1_CH7: Analog output from Galvanic Skin Response divider circuit
#define PIN_BUZZER_OUT  25   // Digital Output: Active Buzzer / Vehicle ECU alarm trigger
#define PIN_I2C_SDA     21   // ESP32 default I2C Data line for MPU6050
#define PIN_I2C_SCL     22   // ESP32 default I2C Clock line for MPU6050

// ─── Algorithmic Parameters & Thresholds (from Research Paper) ─────────────────────────────────
// 1. ECG / HRV (SDNN) parameters
#define ECG_SAMPLE_RATE_HZ     250     // Nyquist-satisfying cardiac sampling rate
#define ECG_SAMPLE_PERIOD_US   4000    // 1 / 250 Hz = 4000 microseconds
#define NN_BUFFER_SIZE         30      // Rolling circular buffer length for NN intervals (N = 30)
#define NN_MIN_PLAUSIBLE_MS    300     // Minimum biologically valid R-R interval (max 200 bpm)
#define NN_MAX_PLAUSIBLE_MS    1500    // Maximum biologically valid R-R interval (min 40 bpm)
#define ECG_PEAK_THRESHOLD_MIN 2200    // Adaptive threshold floor for 12-bit ADC (0-4095)
#define ALPHA_SDNN_OFFSET_MS   45.0f   // SDNN-rise threshold above baseline (alpha = 45 ms)

// 2. GSR / Electrodermal Activity (EDA) parameters
#define GSR_SAMPLE_RATE_HZ     50      // 50 Hz sampling rate
#define GSR_SAMPLE_PERIOD_MS   20      // 1 / 50 Hz = 20 ms
#define SCL_MA_WINDOW_LEN      150     // 3-second moving average window (150 samples @ 50 Hz)
#define SCL_SLOPE_EPOCH_SEC    60      // Slope calculation epoch T = 60 seconds
#define BETA_SLOPE_THRESHOLD   0.50f   // Negative slope threshold magnitude beta = 0.50 uS/s
#define GSR_RAW_ALERT_FLOOR    1200    // Clamping limits to reject electrode disconnection
#define GSR_RAW_CEIL           3800

// 3. MPU6050 IMU Motion Artifact Rejection Gate
#define GYRO_Z_STABLE_THRESH_DEG 40.0f // Angular velocity threshold (highway lane keeping < 25 deg/s)

// 4. Decision & Update loop rate
#define FUSED_LOOP_PERIOD_MS   200     // 5 Hz decision state machine loop

// ─── Module Instances & State Variables ────────────────────────────────────────────────────────
MPU6050 mpu;

// AI Vision inputs (received via USB-UART)
volatile int eyeClosedClassifier = 0;   // 1 when sustained eyelid closure detected (> 333 ms)
volatile int fatigueClassifier   = 0;   // 1 when blink rate >= 25 bpm or long closure >= 1.5 s

// ECG & HRV State
unsigned long lastEcgSampleMicros = 0;
int  ecgPrevRaw = 0;
int  ecgDerivative = 0;
int  ecgAdaptiveThresh = ECG_PEAK_THRESHOLD_MIN;
unsigned long lastRPeakMillis = 0;
float nnBuffer[NN_BUFFER_SIZE];
int   nnCount = 0;
int   nnWriteIdx = 0;
float currentSdnn = 0.0f;
float baselineSdnn = 35.0f;             // Per-driver resting baseline (calibrated or default)
int   currentBpm = 0;

// GSR State
unsigned long lastGsrSampleMillis = 0;
int   gsrRingBuffer[SCL_MA_WINDOW_LEN];
long  gsrWindowSum = 0;
int   gsrRingIdx = 0;
bool  gsrBufferFilled = false;
float currentScl = 0.0f;
float sclHistory[SCL_SLOPE_EPOCH_SEC];  // 1-second historical SCL snapshots
int   sclHistIdx = 0;
int   sclHistCount = 0;
float currentSclSlope = 0.0f;           // Delta sigma / Delta t (uS/s)
float baselineScl = 320.0f;

// IMU State
int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;
float   gyroZDegSec = 0.0f;
bool    imuStable = true;

// Timing & Communication Buffers
unsigned long lastFusedLoopMillis = 0;
unsigned long lastOneSecTick = 0;
String serialRxBuffer = "";
bool   drowsinessAlertActive = false;

// ─── Function Prototypes ───────────────────────────────────────────────────────────────────────
void processEcgSample();
void processGsrSample();
void updateImuTelemetry();
void computeHrvMetrics();
void computeGsrSlope();
void evaluateFusedDecisionLogic();
void parseSerialPayload(const String& msg);
void calibratePerDriverBaseline();

// ===============================================================================================
// SETUP: Hardware initialization, IMU setup, baseline calibration
// ===============================================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_BUZZER_OUT, OUTPUT);
  digitalWrite(PIN_BUZZER_OUT, LOW); // Ensure silent on boot

  // Initialize I2C Bus for MPU6050
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  mpu.initialize();

  // Clear buffers
  for (int i = 0; i < NN_BUFFER_SIZE; i++)    nnBuffer[i] = 0.0f;
  for (int i = 0; i < SCL_MA_WINDOW_LEN; i++) gsrRingBuffer[i] = 0;
  for (int i = 0; i < SCL_SLOPE_EPOCH_SEC; i++) sclHistory[i] = 0.0f;

  Serial.println(F("================================================================="));
  Serial.println(F("[SYSTEM INIT] ESP32 Multi-Modal Drowsiness Monitor Starting..."));
  Serial.print(F("[SYSTEM INIT] MPU6050 Status: "));
  Serial.println(mpu.testConnection() ? F("CONNECTED (OK)") : F("FAIL / NOT DETECTED"));
  Serial.println(F("[SYSTEM INIT] Sampling: ECG @ 250Hz | GSR @ 50Hz | Fused Loop @ 5Hz"));
  Serial.println(F("[SYSTEM INIT] Ready to receive AI Vision packets (EYE:X,FAT:Y)"));
  Serial.println(F("================================================================="));

  // Quick initial baseline stabilization
  calibratePerDriverBaseline();
}

// ===============================================================================================
// MAIN LOOP: Non-blocking deterministic real-time executive
// ===============================================================================================
void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  // ── 1. High-Precision ECG Sampling @ 250 Hz (every 4000 us) ─────────────────
  if (currentMicros - lastEcgSampleMicros >= ECG_SAMPLE_PERIOD_US) {
    lastEcgSampleMicros = currentMicros;
    processEcgSample();
  }

  // ── 2. GSR Sampling @ 50 Hz (every 20 ms) ───────────────────────────────────
  if (currentMillis - lastGsrSampleMillis >= GSR_SAMPLE_PERIOD_MS) {
    lastGsrSampleMillis = currentMillis;
    processGsrSample();
  }

  // ── 3. Parse Inbound Serial ASCII Packets from Laptop AI Vision ──────────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialRxBuffer.length() > 0) {
        serialRxBuffer.trim();
        parseSerialPayload(serialRxBuffer);
        serialRxBuffer = "";
      }
    } else {
      if (serialRxBuffer.length() < 64) {
        serialRxBuffer += c;
      }
    }
  }

  // ── 4. One-Second Tick: Snapshot SCL for Slope Derivation ────────────────────
  if (currentMillis - lastOneSecTick >= 1000) {
    lastOneSecTick = currentMillis;
    sclHistory[sclHistIdx] = currentScl;
    sclHistIdx = (sclHistIdx + 1) % SCL_SLOPE_EPOCH_SEC;
    if (sclHistCount < SCL_SLOPE_EPOCH_SEC) sclHistCount++;
    computeGsrSlope();
  }

  // ── 5. Fused Decision State Machine & Alert Actuation @ 5 Hz (every 200 ms) ──
  if (currentMillis - lastFusedLoopMillis >= FUSED_LOOP_PERIOD_MS) {
    lastFusedLoopMillis = currentMillis;
    updateImuTelemetry();
    computeHrvMetrics();
    evaluateFusedDecisionLogic();
  }
}

// ===============================================================================================
// SIGNAL PROCESSING 1: ECG Pan-Tompkins Peak Detection & NN Interval Estimation
// ===============================================================================================
void processEcgSample() {
  int rawEcg = analogRead(PIN_ECG_IN);

  // Digital first-difference FIR derivative filter
  ecgDerivative = rawEcg - ecgPrevRaw;
  ecgPrevRaw = rawEcg;

  // Pan-Tompkins squared derivative emphasizing high-frequency QRS complex slopes
  long energy = (long)ecgDerivative * (long)ecgDerivative;

  // Adaptive threshold decay to prevent missed beats while suppressing T-wave peaks
  if (ecgAdaptiveThresh > ECG_PEAK_THRESHOLD_MIN) {
    ecgAdaptiveThresh -= 2;
  }

  unsigned long now = millis();
  if (energy > ((long)ecgAdaptiveThresh * 15L) && (now - lastRPeakMillis > NN_MIN_PLAUSIBLE_MS)) {
    if (lastRPeakMillis > 0) {
      float nnInterval = (float)(now - lastRPeakMillis);

      // Motion-artifact rejection: Biological plausibility gate [300 ms - 1500 ms]
      if (nnInterval >= NN_MIN_PLAUSIBLE_MS && nnInterval <= NN_MAX_PLAUSIBLE_MS) {
        nnBuffer[nnWriteIdx] = nnInterval;
        nnWriteIdx = (nnWriteIdx + 1) % NN_BUFFER_SIZE;
        if (nnCount < NN_BUFFER_SIZE) nnCount++;

        currentBpm = (int)(60000.0f / nnInterval);
      }
    }
    lastRPeakMillis = now;
    ecgAdaptiveThresh = max((int)(energy / 20L), ECG_PEAK_THRESHOLD_MIN);
  }
}

// ===============================================================================================
// SIGNAL PROCESSING 2: Rolling Heart Rate Variability (SDNN) Calculation (Equation 1)
// SDNN = sqrt( (1 / N) * sum( (NN_i - mean_NN)^2 ) )
// ===============================================================================================
void computeHrvMetrics() {
  if (nnCount < 5) return; // Need minimal statistics

  float sum = 0.0f;
  for (int i = 0; i < nnCount; i++) {
    sum += nnBuffer[i];
  }
  float meanNn = sum / (float)nnCount;

  float varianceSum = 0.0f;
  for (int i = 0; i < nnCount; i++) {
    float diff = nnBuffer[i] - meanNn;
    varianceSum += diff * diff;
  }
  currentSdnn = sqrtf(varianceSum / (float)nnCount);
}

// ===============================================================================================
// SIGNAL PROCESSING 3: GSR 3-Second Moving Average (Equation 2)
// SCL(t) = (1 / W) * sum( GSR(t - i) )
// ===============================================================================================
void processGsrSample() {
  int rawGsr = analogRead(PIN_GSR_IN);

  // Electrode disconnection fault guard
  if (rawGsr < GSR_RAW_ALERT_FLOOR || rawGsr > GSR_RAW_CEIL) {
    // Hold previous valid reading to reject detachment spikes
    rawGsr = (int)currentScl;
  }

  // Update rolling sum
  gsrWindowSum -= gsrRingBuffer[gsrRingIdx];
  gsrRingBuffer[gsrRingIdx] = rawGsr;
  gsrWindowSum += rawGsr;

  gsrRingIdx++;
  if (gsrRingIdx >= SCL_MA_WINDOW_LEN) {
    gsrRingIdx = 0;
    gsrBufferFilled = true;
  }

  int divisor = gsrBufferFilled ? SCL_MA_WINDOW_LEN : max(1, gsrRingIdx);
  currentScl = (float)gsrWindowSum / (float)divisor;
}

// ===============================================================================================
// SIGNAL PROCESSING 4: Finite-Difference GSR Slope Estimation (Equation 3)
// Delta_sigma / Delta_t = (SCL(t) - SCL(t - T)) / T
// ===============================================================================================
void computeGsrSlope() {
  if (sclHistCount < 10) {
    currentSclSlope = 0.0f;
    return;
  }

  // Oldest sample in circular history
  int oldestIdx = (sclHistCount < SCL_SLOPE_EPOCH_SEC) ? 0 : sclHistIdx;
  float oldestScl = sclHistory[oldestIdx];
  float dt = (float)sclHistCount;

  // Negative slope represents sympathetic arousal decline
  currentSclSlope = (currentScl - oldestScl) / dt;
}

// ===============================================================================================
// SIGNAL PROCESSING 5: MPU6050 Telemetry & Steering Motion-Artifact Gating
// Gating rule: |w_z| <= 40 deg/s => IMU_stable = 1
// ===============================================================================================
void updateImuTelemetry() {
  mpu.getRotation(&rawGx, &rawGy, &rawGz);
  // Scale factor for default +/- 250 deg/s range is 131.0 LSB / (deg/s)
  gyroZDegSec = (float)rawGz / 131.0f;

  imuStable = (fabsf(gyroZDegSec) <= GYRO_Z_STABLE_THRESH_DEG);
}

// ===============================================================================================
// DECISION ENGINE: Deterministic Multimodal Fused Decision State Machine
// ALERT = [EYE_CLOSED = 1]
//      OR [ (FATIGUE = 1 OR SDNN > baseline + alpha) AND Delta_sigma/Delta_t < -beta AND IMU_stable = 1 ]
// ===============================================================================================
void evaluateFusedDecisionLogic() {
  // Path 1: Fast-Path Behavioural Safety Interlock (Zero-latency ocular closure)
  bool fastPathAlert = (eyeClosedClassifier == 1);

  // Path 2: Autonomic Nervous System Physiological Degradation Precursor
  bool parasympatheticDominant = (currentSdnn > (baselineSdnn + ALPHA_SDNN_OFFSET_MS));
  bool sympatheticWithdrawal   = (currentSclSlope < -BETA_SLOPE_THRESHOLD);
  bool aiFatigueFlagged        = (fatigueClassifier == 1);

  bool physiologicalAlert = (parasympatheticDominant || aiFatigueFlagged) 
                            && sympatheticWithdrawal 
                            && imuStable;

  // Final Fused Decision
  drowsinessAlertActive = fastPathAlert || physiologicalAlert;

  // Actuate Hardware Audible Alarm
  digitalWrite(PIN_BUZZER_OUT, drowsinessAlertActive ? HIGH : LOW);

  // Telemetry Output to Serial (115200 baud) for Laptop HUD / ECU logging
  Serial.print(drowsinessAlertActive ? F("[ALERT] ") : F("[SAFE]  "));
  Serial.print(F("EYE="));        Serial.print(eyeClosedClassifier);
  Serial.print(F(" | FAT="));     Serial.print(fatigueClassifier);
  Serial.print(F(" | SDNN="));    Serial.print(currentSdnn, 1);
  Serial.print(F("ms (base="));   Serial.print(baselineSdnn, 1);
  Serial.print(F(") | SCL="));    Serial.print(currentScl, 1);
  Serial.print(F(" | Slope="));   Serial.print(currentSclSlope, 3);
  Serial.print(F("uS/s | GyroZ="));Serial.print(gyroZDegSec, 1);
  Serial.print(F("d/s | IMU_OK="));Serial.print(imuStable ? 1 : 0);
  Serial.print(F(" | BPM="));     Serial.println(currentBpm);
}

// ===============================================================================================
// INBOUND SERIAL PARSER: Decodes "EYE:X,FAT:Y\n" from Laptop AI Vision Layer
// ===============================================================================================
void parseSerialPayload(const String& msg) {
  int eyeIdx = msg.indexOf("EYE:");
  int fatIdx = msg.indexOf("FAT:");

  if (eyeIdx != -1) {
    char c = msg.charAt(eyeIdx + 4);
    eyeClosedClassifier = (c == '1') ? 1 : 0;
  }
  if (fatIdx != -1) {
    char c = msg.charAt(fatIdx + 4);
    fatigueClassifier = (c == '1') ? 1 : 0;
  }
}

// ===============================================================================================
// CALIBRATION: Per-Driver Baseline Protocol (Section 3.4.1)
// ===============================================================================================
void calibratePerDriverBaseline() {
  Serial.println(F("[CALIB] Establishing initial sensor baseline (settling 3 seconds)..."));
  long gsrSum = 0;
  int samples = 0;
  unsigned long start = millis();
  while (millis() - start < 3000) {
    gsrSum += analogRead(PIN_GSR_IN);
    samples++;
    delay(20);
  }
  if (samples > 0) {
    baselineScl = (float)gsrSum / (float)samples;
    currentScl = baselineScl;
  }
  baselineSdnn = 35.0f; // Standard adult resting reference
  Serial.print(F("[CALIB] Baseline SCL set to: ")); Serial.println(baselineScl);
  Serial.print(F("[CALIB] Baseline SDNN set to: ")); Serial.println(baselineSdnn);
}