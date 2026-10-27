#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ─── Pins ─────────────────────────────────────
#define GSR_PIN 35
#define ECG_PIN 34
#define BUZZER_PIN 25

// ─── Thresholds ───────────────────────────────
#define TILT_THRESHOLD 15000   // adjust if too sensitive

// ─── Globals ──────────────────────────────────
String serialBuffer = "";

int eyeClosed = 0;
int fatigue = 0;

int16_t ax, ay, az;

// ─── ECG BPM variables ────────────────────────
unsigned long lastBeatTime = 0;
int bpm = 0;
int prevECG = 0;

// ─── GSR smoothing ────────────────────────────
int smoothGSR(int newVal) {
  static float prev = 0;
  prev = (0.7 * prev) + (0.3 * newVal);
  return (int)prev;
}

// ─── GSR averaging ────────────────────────────
int readGSR() {
  int sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += analogRead(GSR_PIN);
    delay(5);
  }
  return sum / 15;
}

// ─── Normalize GSR ────────────────────────────
int normalizeGSR(int raw) {
  if (raw > 3500) raw = 3500;
  if (raw < 800)  raw = 800;

  return map(raw, 800, 3500, 0, 100);
}

// ─── Classify GSR ─────────────────────────────
int classifyGSR(int raw) {
  int norm = normalizeGSR(raw);

  if (norm > 90) return 0;   // No contact
  if (norm > 40) return 1;   // Active
  return 2;                  // Low arousal (drowsy)
}

// ─── Parse Serial from Python ─────────────────
void parseSerialMessage(String msg) {
  int e = msg.indexOf("EYE:");
  int f = msg.indexOf("FAT:");

  if (e != -1 && f != -1) {
    eyeClosed = msg.substring(e + 4, e + 5).toInt();
    fatigue   = msg.substring(f + 4, f + 5).toInt();
  }
}

// ─── Setup ────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  mpu.initialize();

  Serial.println("SYSTEM READY");
}

// ─── Loop ─────────────────────────────────────
void loop() {

  // ─── 1. Read Serial (Python) ───────────────
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer.trim();
      parseSerialMessage(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }

  // ─── 2. GSR Processing ─────────────────────
  int gsrRaw = readGSR();
  gsrRaw = smoothGSR(gsrRaw);

  int gsrState = classifyGSR(gsrRaw);

  bool validTouch = (gsrState != 0);
  bool lowGSR = (gsrState == 2);

  // ─── 3. ECG Processing ─────────────────────
  int ecgValue = analogRead(ECG_PIN);

  if (ecgValue > 2000 && prevECG <= 2000) {
    unsigned long now = millis();
    if (lastBeatTime != 0) {
      bpm = 60000 / (now - lastBeatTime);
    }
    lastBeatTime = now;
  }
  prevECG = ecgValue;

  bool abnormalECG = (bpm < 50 || bpm > 120);

  // ─── 4. MPU6050 Processing ─────────────────
  mpu.getAcceleration(&ax, &ay, &az);

  bool headTilt = (abs(ax) > TILT_THRESHOLD || abs(ay) > TILT_THRESHOLD);

  // ─── 5. Final Decision Logic ───────────────
  bool drowsy = false;

  if (eyeClosed == 1) drowsy = true;

  if (fatigue == 1 && lowGSR && validTouch) drowsy = true;

  if (headTilt) drowsy = true;

  if (abnormalECG && bpm != 0) drowsy = true;

  // ─── 6. Output ─────────────────────────────
  if (drowsy) {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("[ALERT] DROWSY");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("[SAFE]");
  }

  // ─── Debug Output ──────────────────────────
  Serial.print(" | State: "); Serial.print(gsrState);
  Serial.print(" | BPM: "); Serial.print(bpm);
  Serial.print(" | Tilt: "); Serial.print(headTilt);
  Serial.print(" | Eye: "); Serial.print(eyeClosed);
  Serial.print(" | Fatigue: "); Serial.println(fatigue);

  delay(100);
}