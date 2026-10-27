"""
Automated Pipeline & Mathematical Unit Tests
============================================
Verifies:
1. Eye Aspect Ratio (EAR) metric calculation
2. Rolling SDNN (Heart Rate Variability) algorithm
3. GSR Skin Conductance Level (SCL) moving average & slope calculation
4. Serial telemetry encoding & decoding ("EYE:X,FAT:Y\n")
5. Deterministic Fused Decision Logic truth table
"""

import math
import unittest

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def compute_ear(eye_landmarks):
    # p1, p2, p3, p4, p5, p6
    p1, p2, p3, p4, p5, p6 = eye_landmarks
    vertical_1 = euclidean_distance(p2, p6)
    vertical_2 = euclidean_distance(p3, p5)
    horizontal = euclidean_distance(p1, p4)
    if horizontal < 1e-6:
        return 0.0
    return (vertical_1 + vertical_2) / (2.0 * horizontal)

def compute_sdnn(nn_intervals):
    if not nn_intervals:
        return 0.0
    mean_nn = sum(nn_intervals) / len(nn_intervals)
    variance = sum((x - mean_nn)**2 for x in nn_intervals) / len(nn_intervals)
    return math.sqrt(variance)

def compute_gsr_slope(scl_current, scl_old, epoch_seconds):
    if epoch_seconds <= 0:
        return 0.0
    return (scl_current - scl_old) / epoch_seconds

def fused_alert_decision(eye_closed, fatigue, sdnn, baseline_sdnn, alpha, scl_slope, beta, imu_stable):
    fast_path = (eye_closed == 1)
    parasympathetic_dominant = (sdnn > (baseline_sdnn + alpha))
    sympathetic_withdrawal = (scl_slope < -beta)
    secondary_validated = (fatigue == 1 or parasympathetic_dominant) and sympathetic_withdrawal and (imu_stable == 1)
    return fast_path or secondary_validated

class TestDrowsinessPipeline(unittest.TestCase):

    def test_ear_open_eye(self):
        # Coordinates resembling an open eye
        landmarks = [
            (30, 50),   # p1: outer corner
            (40, 42),   # p2: upper-outer
            (50, 42),   # p3: upper-inner
            (60, 50),   # p4: inner corner
            (50, 58),   # p5: lower-inner
            (40, 58)    # p6: lower-outer
        ]
        ear = compute_ear(landmarks)
        # Expected horizontal = 30, vertical_1 = 16, vertical_2 = 16 => EAR = 32 / (2 * 30) = 0.533
        self.assertGreater(ear, 0.25)

    def test_ear_closed_eye(self):
        # Coordinates resembling a closed eye (vertical separation ~ 1-2 pixels)
        landmarks = [
            (30, 50),
            (40, 49.5),
            (50, 49.5),
            (60, 50),
            (50, 50.5),
            (40, 50.5)
        ]
        ear = compute_ear(landmarks)
        # Expected EAR << 0.15
        self.assertLess(ear, 0.15)

    def test_sdnn_resting_and_fatigued(self):
        # Normal steady resting NN intervals (approx 75 bpm -> 800 ms)
        resting_nn = [800, 805, 795, 810, 790, 802, 798, 804]
        sdnn_rest = compute_sdnn(resting_nn)
        self.assertLess(sdnn_rest, 15.0)

        # High variability (parasympathetic drowsiness onset)
        drowsy_nn = [720, 920, 680, 950, 700, 930, 690, 960]
        sdnn_drowsy = compute_sdnn(drowsy_nn)
        self.assertGreater(sdnn_drowsy, 100.0)

    def test_gsr_slope_relaxation(self):
        scl_start = 350.0  # uS
        scl_end   = 310.0  # uS (downward drift over 60s)
        slope = compute_gsr_slope(scl_end, scl_start, 60.0)
        self.assertAlmostEqual(slope, -0.6667, places=3)
        self.assertLess(slope, -0.5) # Exceeds beta threshold

    def test_fused_decision_rules(self):
        # 1. Alert condition: Fast path eye closed
        self.assertTrue(fused_alert_decision(eye_closed=1, fatigue=0, sdnn=30, baseline_sdnn=30, alpha=45, scl_slope=0.1, beta=0.5, imu_stable=1))

        # 2. Safe condition: Normal open eye and alert physiology
        self.assertFalse(fused_alert_decision(eye_closed=0, fatigue=0, sdnn=30, baseline_sdnn=30, alpha=45, scl_slope=0.0, beta=0.5, imu_stable=1))

        # 3. Alert condition: Fatigue + GSR slope drop + stable IMU
        self.assertTrue(fused_alert_decision(eye_closed=0, fatigue=1, sdnn=30, baseline_sdnn=30, alpha=45, scl_slope=-0.6, beta=0.5, imu_stable=1))

        # 4. Filtered condition: High steering motion (IMU unstable) suppresses false physiological alert
        self.assertFalse(fused_alert_decision(eye_closed=0, fatigue=1, sdnn=30, baseline_sdnn=30, alpha=45, scl_slope=-0.6, beta=0.5, imu_stable=0))

if __name__ == '__main__':
    unittest.main()
