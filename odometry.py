# Import Libraries
from a_star import AStar
import time
import math
from smbus2 import SMBus

# Define the robot
romi = AStar()

# ---------------- ENCODERS ---------------- #

class RomiEncoder:
    def __init__(self, robot):
        self.robot = robot
        self.offset_left = 0
        self.offset_right = 0
        self.last_left = 0
        self.last_right = 0

    def reset(self):
        raw = self.robot.read_encoders()
        self.offset_left = raw[0]
        self.offset_right = raw[1]
        self.last_left = 0
        self.last_right = 0

    def get_counts(self):
        try:
            raw = self.robot.read_encoders()
        except OSError:
            return self.last_left, self.last_right

        rel_left = raw[0] - self.offset_left
        rel_right = raw[1] - self.offset_right

        # Handle rollover
        if rel_left > 32768: rel_left -= 65536
        if rel_left < -32768: rel_left += 65536
        if rel_right > 32768: rel_right -= 65536
        if rel_right < -32768: rel_right += 65536

        # Sanity filter
        if abs(rel_left - self.last_left) > 1000 or abs(rel_right - self.last_right) > 1000:
            return self.last_left, self.last_right

        self.last_left = rel_left
        self.last_right = rel_right
        return rel_left, rel_right

encoder_helper = RomiEncoder(romi)

# ---------------- ODOMETRY ---------------- #

class Odometry:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

        self.last_left = 0
        self.last_right = 0

        self.WHEEL_BASE_MM = 135

    def update(self, left_counts, right_counts):
        left_mm = left_counts * MM_PER_COUNT
        right_mm = right_counts * MM_PER_COUNT

        deltaL = left_mm - self.last_left
        deltaR = right_mm - self.last_right

        self.last_left = left_mm
        self.last_right = right_mm

        deltaCenter = (deltaL + deltaR) / 2
        deltaTheta = (deltaR - deltaL) / self.WHEEL_BASE_MM

        self.theta += deltaTheta

        #accumulate position
        self.x += deltaCenter * math.cos(self.theta)
        self.y += deltaCenter * math.sin(self.theta)

odom = Odometry()

# ---------------- GYRO ---------------- #

LSM6_ADDR = 0x6b
CTRL2_G = 0x11
OUTZ_L = 0x26

bus = SMBus(1)
time.sleep(0.5)  # allow IMU to boot

# Safe initialization
for _ in range(5):
    try:
        bus.write_byte_data(LSM6_ADDR, CTRL2_G, 0x8C)
        break
    except OSError:
        time.sleep(0.1)

time.sleep(0.2)

last_valid_gyro = 0

def get_gyro_z_raw():
    global last_valid_gyro

    for _ in range(3):
        try:
            data = bus.read_i2c_block_data(LSM6_ADDR, OUTZ_L, 2)
            raw_z = data[0] | (data[1] << 8)
            if raw_z > 32767:
                raw_z -= 65536

            last_valid_gyro = raw_z
            return raw_z

        except OSError:
            time.sleep(0.001)

    return last_valid_gyro

# ---------------- CONSTANTS ---------------- #

WHEEL_DIAMETER_MM = 70.0
COUNTS_PER_REV = 1440
MM_PER_COUNT = (WHEEL_DIAMETER_MM * math.pi) / COUNTS_PER_REV

GYRO_SCALE = 0.070

# ---------------- CALIBRATION ---------------- #

def calibrate_gyro(samples=100):
    print("Calibrating gyro... do not move the robot.")
    total = 0
    count = 0

    while count < samples:
        try:
            total += get_gyro_z_raw()
            count += 1
        except OSError:
            continue

        time.sleep(0.02)

    print("Calibration done.")
    return total / samples

GYRO_BIAS = calibrate_gyro()

# ---------------- DRIVE ---------------- #

def drive_to_point(robot, odom, target_x, target_y):
    encoder_helper.reset()
    odom.last_left = 0
    odom.last_right = 0

    kP_angle = 2.5
    kP_distance = 0.8

    while True:
        try:
            left, right = encoder_helper.get_counts()
            odom.update(left, right)

            dx = target_x - odom.x
            dy = target_y - odom.y

            distance = math.sqrt(dx**2 + dy**2)

            print(f"x:{odom.x:.1f} y:{odom.y:.1f} dist:{distance:.1f}")

            if distance < 10:
                break

            target_theta = math.atan2(dy, dx)
            angle_error = target_theta - odom.theta

            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            forward = kP_distance * distance
            turn = kP_angle * angle_error

            # prevent wide arcs
            if abs(angle_error) > 0.5:
                forward = 0

            left_speed = int(forward - turn)
            right_speed = int(forward + turn)

            # ✅ FIXED clamp
            left_speed = max(min(left_speed, 300), -300)
            right_speed = max(min(right_speed, 300), -300)

            robot.motors(left_speed, right_speed)

            time.sleep(0.015)

        except OSError:
            continue

    robot.motors(0, 0)

# ---------------- RUN ---------------- #

drive_to_point(romi, odom, 500, 0)
drive_to_point(romi, odom, 500, 500)