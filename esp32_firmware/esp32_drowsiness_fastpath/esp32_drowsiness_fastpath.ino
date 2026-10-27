// ============================================================
//  Driver Drowsiness Detection — ESP32 Side
//  Board  : ESP32 Dev Module
//  Sensor : GSR on GPIO35 (analog)
//  Output : Active buzzer on GPIO25
//  Serial : Receives "EYE:1,FAT:0" style strings from laptop
// ============================================================

// ─── Pin definitions ────────────────────────────────────────
const int GSR_PIN    = 35;   // ADC1_CH7 — analog GSR input
const int BUZZER_PIN = 25;   // Active buzzer (HIGH = ON)

// ─── GSR threshold ──────────────────────────────────────────
// ESP32 ADC is 12-bit (0–4095).
// A LOW raw value → low skin conductance → drowsy / low arousal.
// Calibrate for your sensor; 1800 is a reasonable demo starting point.
const int GSR_LOW_AROUSAL_THRESHOLD = 1800;

// ─── Decision state ─────────────────────────────────────────
int eyeClosed = 0;   // 1 = eyes closed (from laptop vision model)
int fatigue   = 0;   // 1 = fatigue detected (from laptop vision model)

// ─── Serial parse buffer ────────────────────────────────────
String serialBuffer = "";

// ─── Simulated ECG (demo only — NOT used in decision logic) ─
// The ECG sensor is faulty; these values are purely illustrative.
int simulatedECG = 0;

// ============================================================
void setup() {
  Serial.begin(115200);           // Must match pyserial baud rate
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Buzzer OFF at startup

  // GSR pin is input-only; analogRead works without pinMode on ESP32
  Serial.println("[ESP32] Drowsiness monitor ready.");
  Serial.println("[ESP32] Waiting for vision data from laptop...");
}

// ============================================================
void loop() {

  // ── 1. Read GSR ──────────────────────────────────────────
  int gsrRaw = analogRead(GSR_PIN);
  // Low value → low arousal (driver may be drowsy)
  bool gsrLowArousal = (gsrRaw < GSR_LOW_AROUSAL_THRESHOLD);

  // ── 2. Simulate ECG (display only, not in decision logic) ─
  simulatedECG = random(60, 100);  // Fake BPM for demo dashboard

  // ── 3. Parse incoming serial from laptop ─────────────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {               // End of message
      parseSerialMessage(serialBuffer.trim());
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }

  // ── 4. Decision logic ────────────────────────────────────
  bool buzzerOn = false;

  // A) IMMEDIATE SAFETY TRIGGER — eyes closed
  if (eyeClosed == 1) {
    buzzerOn = true;
    Serial.println("[ALERT] Eye closure detected → BUZZER ON");
  }

  // B) SECONDARY TRIGGER — fatigue confirmed by both AI and GSR
  if (fatigue == 1 && gsrLowArousal) {
    buzzerOn = true;
    Serial.println("[ALERT] Fatigue + low GSR arousal → BUZZER ON");
  }

  // C) Safe — buzzer off
  if (!buzzerOn) {
    Serial.println("[OK] Driver alert — BUZZER OFF");
  }

  // ── 5. Drive buzzer ──────────────────────────────────────
  digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);

  // ── 6. Debug output ──────────────────────────────────────
  Serial.print("[DATA] GSR=");
  Serial.print(gsrRaw);
  Serial.print(" | LowArousal=");
  Serial.print(gsrLowArousal ? "YES" : "NO");
  Serial.print(" | EYE=");
  Serial.print(eyeClosed);
  Serial.print(" | FAT=");
  Serial.print(fatigue);
  Serial.print(" | ECG(sim)=");
  Serial.print(simulatedECG);
  Serial.println(" BPM");

  delay(200);  // 5 Hz update rate — fast enough for safety
}

// ============================================================
// parseSerialMessage()
// Expects format: "EYE:1,FAT:0"
// Tolerates extra whitespace; ignores malformed packets.
// ============================================================
void parseSerialMessage(String msg) {
  if (msg.length() == 0) return;

  // ── Extract EYE value ────────────────────────────────────
  int eyeIdx = msg.indexOf("EYE:");
  if (eyeIdx != -1) {
    // Character right after "EYE:" is the digit
    char eyeChar = msg.charAt(eyeIdx + 4);
    eyeClosed = (eyeChar == '1') ? 1 : 0;
  }

  // ── Extract FAT value ────────────────────────────────────
  int fatIdx = msg.indexOf("FAT:");
  if (fatIdx != -1) {
    char fatChar = msg.charAt(fatIdx + 4);
    fatigue = (fatChar == '1') ? 1 : 0;
  }

  Serial.print("[RECV] ");
  Serial.println(msg);
}
