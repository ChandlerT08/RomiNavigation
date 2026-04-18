# Import Libraries
from a_star import AStar
import time
import math
from smbus2 import SMBus
import threading

# Define the robot
romi = AStar()

# Create an I2C lock to prevent a gyro and encoder collision on the bus
i2c_lock = threading.Lock()

# ---------------- ENCODERS ---------------- #

class RomiEncoder:
    def __init__(self, robot):
        self.robot = robot
        self.total_left = 0
        self.total_right = 0
        self.last_left = 0
        self.last_right = 0
        self.fail_count = 0
        self.MAX_FAILS = 5

    def reset(self):
        raw = self.robot.read_encoders()
        self.offset_left = raw[0]
        self.offset_right = raw[1]
        self.last_left = None
        self.last_right = None

    def get_counts(self):
        try:
            with i2c_lock:
                raw = self.robot.read_encoders()
            self.fail_count = 0
        except OSError as e:
            self.fail_count += 1
            print(f"[ENC] OSError errno={e.errno} fail#{self._fail_count}")

            if self.fail_count >= self.MAX_FAILS:
                self.recover_bus()
            return self.total_left, self.total_right

        left, right = raw

        print(f"[ENC] raw=({raw[0]},{raw[1]}) rel=({self.total_left},{self.total_right}) stale=({self.last_left},{self.last_right})")

        # First read initialization
        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            return 0, 0

        # Compute delta
        delta_left = left - self.last_left
        delta_right = right - self.last_right

        # Handle wraparound (16-bit encoder assumed)
        if delta_left > 32768: delta_left -= 65536
        if delta_left < -32768: delta_left += 65536
        if delta_right > 32768: delta_right -= 65536
        if delta_right < -32768: delta_right += 65536

        # Sanity filter (now on DELTA, not absolute)
        if abs(delta_left) > 1000 or abs(delta_right) > 1000:
            print("[ENC] Sanity filter triggered!")
            
            delta_left = 0
            delta_right = 0

        

        # Accumulate
        self.total_left += delta_left
        self.total_right += delta_right

        self.last_left = left
        self.last_right = right

        print(f"[ENC] raw=({left},{right}) delta=({delta_left},{delta_right}) total=({self.total_left},{self.total_right})")

        return self.total_left, self.total_right

    def recover_bus(self):
        global bus
        self.fail_count = 0

        # reset bus
        try:
            bus.close()
        except Exception:
            pass
        time.sleep(0.1)
        bus = SMBus(1)
        time.sleep(0.2)

        # reset internal I2C
        try:
            romi = AStar()
            self.robot = romi
            time.sleep(0.2)
        except Exception as e:
            print(f"[Encoder] AStar reinit failed: {e}")

    print("[Encoder] Recovery complete.")

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
            with i2c_lock:
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

# ---------------- CONSTANTS ---------------- #
MOTOR_TRIM = 10  # add to left motor to compensate veer; tune this value

def align_to_target(robot, odom, target_x, target_y, tolerance=0.08):
    """Spin in place until heading matches target direction."""
    print("[Align] Aligning heading...")
    kP = 3.0

    for _ in range(200):  # max ~3 seconds
        try:
            left, right = encoder_helper.get_counts()
            odom.update(left, right)

            dx = target_x - odom.x
            dy = target_y - odom.y
            target_theta = math.atan2(dy, dx)

            angle_error = target_theta - odom.theta
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_error) < tolerance:
                break

            turn = int(kP * angle_error * 80)
            turn = max(min(turn, 150), -150)

            robot.motors(-turn, turn)  # spin in place
            time.sleep(0.015)

        except OSError:
            continue

    robot.motors(0, 0)
    time.sleep(0.1)
    print("[Align] Done.")


def drive_to_point(robot, odom, target_x, target_y):
    odom.last_left = 0
    odom.last_right = 0

    kP_angle = 2.5
    kP_distance = 0.8

    last_distance = float('inf')
    stall_count = 0
    MAX_STALL = 60  # ~0.9 seconds of no progress

    while True:
        try:
            left, right = encoder_helper.get_counts()
            odom.update(left, right)

            dx = target_x - odom.x
            dy = target_y - odom.y
            distance = math.sqrt(dx**2 + dy**2)

            print(f"x:{odom.x:.1f} y:{odom.y:.1f} dist:{distance:.1f}")

            if distance > 15:
                forward = kP_distance * distance
                forward = max(forward, 60)
            else:
                forward = 0

            if distance < 15:
                break

            # ---- Stall detection ----
            if abs(distance - last_distance) < 0.5:
                stall_count += 1
            else:
                stall_count = 0
            last_distance = distance

            if stall_count >= MAX_STALL:
                print("[WARN] No progress detected — stopping.")
                break

            target_theta = math.atan2(dy, dx)
            angle_error = target_theta - odom.theta
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi


            # Scale back forward speed proportionally to angle error
            # instead of a hard cutoff — avoids the oscillation trap
            angle_factor = max(0.0, 1.0 - abs(angle_error) / 0.8)
            forward *= angle_factor

            turn = kP_angle * angle_error

            left_speed  = int(forward - turn) + MOTOR_TRIM
            right_speed = int(forward + turn)

            left_speed  = max(min(left_speed,  300), -300)
            right_speed = max(min(right_speed, 300), -300)

            robot.motors(left_speed, right_speed)
            time.sleep(0.015)

        except OSError:
            continue

    robot.motors(0, 0)

# ---------------- RUN ---------------- #

#Zero encoders then run the functions
encoder_helper.reset()


drive_to_point(romi, odom, 500, 0)
drive_to_point(romi, odom, 500, 500)