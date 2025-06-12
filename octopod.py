import time
import math
import numpy as np
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# === SERVO CONFIGURATION ===
NUM_SERVOS = 24  # 8 legs x 3 servos
SERVOS_PER_LEG = 3

# Each leg has [coxa, femur, tibia] channels
LEG_CHANNELS = {
    "leg1": [0, 1, 2],
    "leg2": [3, 4, 5],
    "leg3": [6, 7, 8],
    "leg4": [9, 10, 11],
    "leg5": [12, 13, 14],
    "leg6": [15, 16, 17],
    "leg7": [18, 19, 20],
    "leg8": [21, 22, 23]
}

# === LEG GEOMETRY (mm) ===
COXA_LENGTH = 40.0
FEMUR_LENGTH = 60.0
TIBIA_LENGTH = 80.0

# === BODY OFFSETS (for each leg) ===
LEG_OFFSETS = {
    "leg1": [100, 50, 0],   # Front Left
    "leg2": [100, 0, 0],
    "leg3": [100, -50, 0],  # Front Right
    "leg4": [0, -75, 0],
    "leg5": [-100, -50, 0], # Back Right
    "leg6": [-100, 0, 0],
    "leg7": [-100, 50, 0],  # Back Left
    "leg8": [0, 75, 0]
}

# === GAIT CONFIGURATION ===
STEP_SIZE = 30
LIFT_HEIGHT = 30
GROUND_HEIGHT = 0
STEP_TIME = 0.3

# === TRIPLET GROUPING FOR TRIPOD GAIT ===
TRIPOD_A = ["leg1", "leg3", "leg5", "leg7"]
TRIPOD_B = ["leg2", "leg4", "leg6", "leg8"]


class LegIK:
    def __init__(self, coxa_length, femur_length, tibia_length):
        self.coxa = coxa_length
        self.femur = femur_length
        self.tibia = tibia_length

    def calculate_angles(self, x, y, z):
        try:
            coxa_angle = math.degrees(math.atan2(y, x)) if x != 0 else 0

            x_adj = math.sqrt(x**2 + y**2) - self.coxa
            leg_length = math.sqrt(x_adj**2 + z**2)

            theta1 = math.acos((self.femur**2 + z**2 - self.tibia**2) / (2 * self.femur * leg_length))
            theta2 = math.atan2(z, x_adj)
            femur_angle = math.degrees(theta1 + theta2)

            theta3 = math.acos((self.femur**2 + self.tibia**2 - leg_length**2) / (2 * self.femur * self.tibia))
            tibia_angle = math.degrees(theta3)

            return [
                np.clip(coxa_angle, 0, 180),
                np.clip(femur_angle, 0, 180),
                np.clip(tibia_angle, 0, 180)
            ]
        except Exception as e:
            print(f"[ERROR] Failed to compute IK: {e}")
            return [90, 90, 90]


class OctoSpider:
    def __init__(self):
        # Initialize I2C bus and PCA9685 object
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50  # Standard servo frequency

        self.ik_solver = LegIK(COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH)

    def set_servo_angle(self, channel, angle):
        pulse_width = int(4096 * ((angle * 11) + 500) / 20000)
        self.pca.channels[channel].duty_cycle = pulse_width

    def move_leg(self, leg_name, target_x, target_y, target_z):
        offset = np.array(LEG_OFFSETS[leg_name])
        x, y, z = np.array([target_x, target_y, target_z]) + offset

        angles = self.ik_solver.calculate_angles(x, y, z)
        channels = LEG_CHANNELS[leg_name]

        for ch, angle in zip(channels, angles):
            self.set_servo_angle(ch, angle)

    def tripod_gait_step(self, step_x, step_y=0):
        # Move body forward
        for group in [TRIPOD_A, TRIPOD_B]:
            # Lift group
            for leg in group:
                self.move_leg(leg, step_x, step_y, LIFT_HEIGHT)
            time.sleep(STEP_TIME)

            # Place group down
            for leg in group:
                self.move_leg(leg, step_x, step_y, GROUND_HEIGHT)
            time.sleep(STEP_TIME)

    def walk_forward(self, steps=10, step_size=STEP_SIZE):
        print("[INFO] Walking forward...")
        for i in range(steps):
            self.tripod_gait_step(step_size)
            print(f"Step {i+1}/{steps}")

    def reset_position(self):
        print("[INFO] Resetting position...")
        for leg in LEG_CHANNELS:
            self.move_leg(leg, 0, 0, GROUND_HEIGHT)
            time.sleep(0.05)
        time.sleep(1)


if __name__ == "__main__":
    spider = OctoSpider()
    spider.reset_position()

    try:
        spider.walk_forward(steps=20)
    except KeyboardInterrupt:
        print("[INFO] Stopping...")
    finally:
        spider.reset_position()
