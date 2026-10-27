"""
Driver Drowsiness Detection — Laptop Side
==========================================
Uses:
  - MediaPipe Face Mesh  → precise eye landmark detection
  - OpenCV               → webcam capture + on-screen display
  - pyserial             → send EYE/FATIGUE signals to ESP32

Install dependencies:
  pip install opencv-python mediapipe pyserial

How it works:
  1. Compute Eye Aspect Ratio (EAR) from 6 landmarks per eye.
     EAR < EAR_THRESHOLD  →  eye is closed.
  2. EYE_CLOSED signal:
       If both eyes have been closed for >= CLOSED_CONSEC_FRAMES
       consecutive frames → EYE_CLOSED = 1.
  3. FATIGUE signal:
       Track blink rate over a rolling BLINK_WINDOW_SECONDS window.
       If blinks per minute >= BLINK_RATE_FATIGUE_THRESH
       OR a single closure lasts >= LONG_CLOSURE_SECONDS
       → FATIGUE = 1.
  4. Serial string "EYE:X,FAT:Y\n" sent to ESP32 every frame.
"""

import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports
import time
import collections
import sys
import math
import threading
import re

# ─── Configuration ─────────────────────────────────────────────────────────────

SERIAL_PORT   = "COM3"     # Change to your ESP32 port (Linux: "/dev/ttyUSB0")
BAUD_RATE     = 115200

# Eye Aspect Ratio threshold — below this the eye is considered closed.
# Typical open-eye EAR ≈ 0.25–0.30; calibrate per user if needed.
EAR_THRESHOLD = 0.22

# How many consecutive frames eyes must be closed to trigger EYE_CLOSED = 1.
# At 30 fps, 10 frames ≈ 333 ms — long enough to ignore normal blinks.
CLOSED_CONSEC_FRAMES = 10

# Fatigue by blink rate: if blinks per minute exceed this, driver is fatigued.
# Normal = ~15–20 bpm; fatigue manifests as >25 bpm (micro-sleeps) or <10 bpm
# (prolonged stare). We use high rate as primary indicator here.
BLINK_RATE_FATIGUE_THRESH = 25   # blinks per minute

# Fatigue by prolonged closure: if eyes stay shut > this many seconds → FATIGUE.
LONG_CLOSURE_SECONDS = 1.5

# Rolling window for blink rate calculation (seconds)
BLINK_WINDOW_SECONDS = 60

# Once FATIGUE fires, hold it at 1 for this many seconds.
# Fixes the bug where FAT drops to 0 before the serial packet is sent.
FATIGUE_HOLD_SECONDS = 3.0

# GSR threshold — must match the value in esp32_drowsiness.ino.
# HIGH ADC value = high resistance = low conductance = drowsy.
# LOW  ADC value = low resistance  = high conductance = alert.
# Observed: alert ≈ 1500–1600, drowsy/loose ≈ 2400. Midpoint = 2000.
GSR_LOW_AROUSAL_THRESHOLD = 2000
GSR_MAX_ADC               = 4095   # ESP32 12-bit ADC

# ─── MediaPipe landmark indices for left / right eye ──────────────────────────
# These 6 points per eye are used for the EAR calculation.
# Reference: MediaPipe Face Mesh topology.
LEFT_EYE_IDX  = [362, 385, 387, 263, 373, 380]
RIGHT_EYE_IDX = [33,  160, 158, 133, 153, 144]

# ─── Helpers ───────────────────────────────────────────────────────────────────

def euclidean(p1, p2):
    """2-D Euclidean distance between two (x, y) tuples."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def eye_aspect_ratio(landmarks, eye_indices, img_w, img_h):
    """
    Compute EAR for one eye.

    EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)

    Landmark layout (index order in eye_indices list):
        p1 = outer corner  (index 0)
        p2 = upper-outer   (index 1)
        p3 = upper-inner   (index 2)
        p4 = inner corner  (index 3)
        p5 = lower-inner   (index 4)
        p6 = lower-outer   (index 5)
    """
    def lm(i):
        pt = landmarks[eye_indices[i]]
        return (pt.x * img_w, pt.y * img_h)

    p1, p2, p3, p4, p5, p6 = lm(0), lm(1), lm(2), lm(3), lm(4), lm(5)

    vertical_1 = euclidean(p2, p6)
    vertical_2 = euclidean(p3, p5)
    horizontal = euclidean(p1, p4)

    if horizontal < 1e-6:
        return 0.0
    return (vertical_1 + vertical_2) / (2.0 * horizontal)


def gsr_reader_thread(ser, gsr_state):
    """
    Background thread: continuously reads lines from the ESP32 serial output
    and extracts the GSR ADC value from lines like:
        [DATA] GSR=1523 | LowArousal=YES | ...
    Updates gsr_state dict (thread-safe via lock).
    """
    while gsr_state["running"]:
        try:
            if ser and ser.is_open:
                raw = ser.readline()
                line = raw.decode("utf-8", errors="ignore").strip()
                m = re.search(r"GSR=(\d+)", line)
                if m:
                    with gsr_state["lock"]:
                        gsr_state["value"] = int(m.group(1))
        except Exception:
            time.sleep(0.05)


def draw_gsr_bar(frame, gsr_value, threshold=GSR_LOW_AROUSAL_THRESHOLD):
    """
    Draw a live GSR level bar at the bottom of the frame.
    - Bar fills left-to-right proportional to ADC value (0–4095)
    - Green  = above threshold (alert / high arousal)
    - Red    = below threshold (low arousal / drowsy)
    - Yellow dashed line marks the threshold position
    """
    h, w = frame.shape[:2]

    BAR_H      = 22     # bar height in pixels
    BAR_MARGIN = 10     # left/right margin
    BAND_H     = 50     # total dark band height at bottom

    # Dark background band
    cv2.rectangle(frame, (0, h - BAND_H), (w, h), (25, 25, 25), -1)

    bar_x  = BAR_MARGIN
    bar_y  = h - BAND_H + 14
    bar_w  = w - 2 * BAR_MARGIN

    # Empty bar background
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + BAR_H),
                  (60, 60, 60), -1)

    # Filled portion
    fill_w    = int(bar_w * min(gsr_value, GSR_MAX_ADC) / GSR_MAX_ADC)
    low_arousal = gsr_value > threshold          # HIGH value = drowsy
    bar_color = (0, 80, 220) if low_arousal else (0, 200, 80)   # blue-red if drowsy, green if alert
    if fill_w > 0:
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + BAR_H),
                      bar_color, -1)

    # Threshold marker line (yellow)
    thresh_x = bar_x + int(bar_w * threshold / GSR_MAX_ADC)
    cv2.line(frame, (thresh_x, bar_y - 4), (thresh_x, bar_y + BAR_H + 4),
             (0, 230, 230), 2)

    # Bar border
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + BAR_H),
                  (120, 120, 120), 1)

    # Labels
    state_text = "LOW AROUSAL (drowsy)" if low_arousal else "ALERT (aroused)"
    state_color = (80, 130, 255) if low_arousal else (0, 200, 80)
    cv2.putText(frame,
                f"GSR: {gsr_value}  [{state_text}]",
                (bar_x, bar_y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, state_color, 1)

    # Threshold label near the marker line
    cv2.putText(frame, f"Thr:{threshold}",
                (thresh_x + 4, bar_y + BAR_H - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 230, 230), 1)


def find_esp32_port():
    """
    Auto-detect the ESP32 serial port by scanning connected devices.
    Falls back to SERIAL_PORT constant if nothing is found.
    """
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["cp210", "ch340", "uart", "esp32", "usb serial"]):
            print(f"[Serial] Auto-detected ESP32 on {p.device}")
            return p.device
    print(f"[Serial] No ESP32 auto-detected — using configured port {SERIAL_PORT}")
    return SERIAL_PORT


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    # ── Serial setup ──────────────────────────────────────────────────────────
    port = find_esp32_port()
    ser  = None
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)   # Wait for ESP32 to finish resetting after connection
        print(f"[Serial] Connected to {port} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"[Serial] WARNING: Could not open {port}: {e}")
        print("[Serial] Running in NO-SERIAL mode (display only).")

    # ── GSR reader thread ─────────────────────────────────────────────────────
    # Shared state dict — the background thread writes, the main loop reads.
    gsr_state = {
        "value":   2048,          # default mid-range until first reading
        "running": True,
        "lock":    threading.Lock(),
    }
    reader = threading.Thread(
        target=gsr_reader_thread,
        args=(ser, gsr_state),
        daemon=True,              # dies automatically when main thread exits
    )
    reader.start()
    print("[Serial] GSR reader thread started.")

    # ── MediaPipe Face Mesh ───────────────────────────────────────────────────
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh    = mp_face_mesh.FaceMesh(
        max_num_faces       = 1,
        refine_landmarks    = True,   # Enables iris tracking (improves eye landmarks)
        min_detection_confidence = 0.6,
        min_tracking_confidence  = 0.6,
    )

    # ── Webcam ───────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[Camera] ERROR: Cannot open webcam.")
        sys.exit(1)

    print("[Camera] Webcam opened. Press 'q' to quit.")

    # ── State variables ───────────────────────────────────────────────────────
    closed_frame_count   = 0       # Consecutive frames with eyes closed
    eye_closed_signal    = 0       # 1 when sustained closure detected
    fatigue_signal       = 0       # 1 when fatigue heuristics fire
    fatigue_last_trigger = 0.0     # timestamp of last fatigue onset (for hold)

    in_blink             = False   # Are we currently tracking a blink?
    closure_start_time   = None    # When current closure started
    blink_timestamps     = collections.deque()  # Rolling blink history

    last_serial_time     = 0.0
    SERIAL_INTERVAL      = 0.1    # Send to ESP32 at 10 Hz max

    # ── Main loop ─────────────────────────────────────────────────────────────
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[Camera] Failed to grab frame.")
            break

        frame = cv2.flip(frame, 1)   # Mirror for natural feel
        h, w  = frame.shape[:2]
        now   = time.time()

        # Convert BGR → RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results   = face_mesh.process(rgb_frame)

        eyes_detected = False
        avg_ear       = 0.0

        if results.multi_face_landmarks:
            lms = results.multi_face_landmarks[0].landmark

            left_ear  = eye_aspect_ratio(lms, LEFT_EYE_IDX,  w, h)
            right_ear = eye_aspect_ratio(lms, RIGHT_EYE_IDX, w, h)
            avg_ear   = (left_ear + right_ear) / 2.0
            eyes_detected = True

            # ── EAR-based closure tracking ────────────────────────────────
            if avg_ear < EAR_THRESHOLD:
                closed_frame_count += 1

                # Track closure start time for long-closure fatigue
                if not in_blink:
                    in_blink           = True
                    closure_start_time = now
            else:
                if in_blink:
                    # Eyes just reopened → record a completed blink
                    blink_timestamps.append(now)
                    in_blink = False

                closed_frame_count = 0

            # ── EYE_CLOSED signal ─────────────────────────────────────────
            eye_closed_signal = 1 if (closed_frame_count >= CLOSED_CONSEC_FRAMES) else 0

            # ── FATIGUE signal ────────────────────────────────────────────
            # Purge blinks older than the rolling window
            cutoff = now - BLINK_WINDOW_SECONDS
            while blink_timestamps and blink_timestamps[0] < cutoff:
                blink_timestamps.popleft()

            # Extrapolate blink rate to blinks-per-minute
            elapsed_window  = min(BLINK_WINDOW_SECONDS, now - (blink_timestamps[0] if blink_timestamps else now))
            if elapsed_window > 5:   # Need at least 5 s of data to estimate
                bpm = (len(blink_timestamps) / elapsed_window) * 60.0
            else:
                bpm = 0.0

            # Long-closure check
            long_closure = (in_blink and closure_start_time is not None
                            and (now - closure_start_time) >= LONG_CLOSURE_SECONDS)

            # ── Fatigue latch fix ─────────────────────────────────────────
            # Once fatigue fires, hold FAT=1 for FATIGUE_HOLD_SECONDS so the
            # serial packet always catches it (fixes FAT showing 0 on ESP32).
            if bpm >= BLINK_RATE_FATIGUE_THRESH or long_closure:
                fatigue_signal       = 1
                fatigue_last_trigger = now          # reset hold timer
            elif now - fatigue_last_trigger < FATIGUE_HOLD_SECONDS:
                fatigue_signal = 1                  # still within hold window
            else:
                fatigue_signal = 0

            # ── Draw eye landmarks ────────────────────────────────────────
            for idx in LEFT_EYE_IDX + RIGHT_EYE_IDX:
                pt = lms[idx]
                cx, cy = int(pt.x * w), int(pt.y * h)
                cv2.circle(frame, (cx, cy), 2, (0, 255, 0), -1)

        # ── Send serial data to ESP32 ──────────────────────────────────────
        if now - last_serial_time >= SERIAL_INTERVAL:
            message = f"EYE:{eye_closed_signal},FAT:{fatigue_signal}\n"
            if ser and ser.is_open:
                try:
                    ser.write(message.encode("utf-8"))
                except serial.SerialException as e:
                    print(f"[Serial] Write error: {e}")
            last_serial_time = now

        # ── Read latest GSR from background thread ────────────────────────
        with gsr_state["lock"]:
            current_gsr = gsr_state["value"]

        # ── HUD overlay ───────────────────────────────────────────────────
        status_color = (0, 200, 80)
        status_text  = "DRIVER SAFE"

        if eye_closed_signal == 1:
            status_color = (0, 0, 255)
            status_text  = "!!! EYES CLOSED !!!"
        elif fatigue_signal == 1 and current_gsr < GSR_LOW_AROUSAL_THRESHOLD:
            status_color = (0, 80, 255)
            status_text  = "!! FATIGUE + LOW AROUSAL !!"
        elif fatigue_signal == 1:
            status_color = (0, 180, 255)
            status_text  = "! FATIGUE (GSR OK)"

        # Top dark band
        cv2.rectangle(frame, (0, 0), (w, 95), (30, 30, 30), -1)

        cv2.putText(frame, status_text, (10, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)

        cv2.putText(frame, f"EAR: {avg_ear:.3f}",
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, f"Closed frames: {closed_frame_count}",
                    (200, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, f"Blinks: {len(blink_timestamps)}",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

        eye_label = "EYE:CLOSED" if eye_closed_signal else "EYE:OPEN"
        fat_label = "FAT:YES"    if fatigue_signal     else "FAT:NO"
        signal_color = (0, 0, 255) if eye_closed_signal or fatigue_signal else (0, 200, 80)
        cv2.putText(frame, f"{eye_label}  {fat_label}",
                    (w - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, signal_color, 2)

        # ── GSR bar at the bottom ─────────────────────────────────────────
        draw_gsr_bar(frame, current_gsr)

        if not eyes_detected:
            cv2.putText(frame, "No face detected", (10, h - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.imshow("Driver Drowsiness Monitor", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[System] Quit key pressed. Shutting down...")
            break

    # ── Cleanup ───────────────────────────────────────────────────────────────
    gsr_state["running"] = False   # signal reader thread to stop
    cap.release()
    face_mesh.close()
    if ser and ser.is_open:
        # Send a safe signal before closing
        try:
            ser.write(b"EYE:0,FAT:0\n")
        except Exception:
            pass
        ser.close()
    cv2.destroyAllWindows()
    print("[System] Closed cleanly.")


if __name__ == "__main__":
    main()
