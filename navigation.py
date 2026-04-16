# Import Libraries
from a_star import AStar
import time
import math
from smbus2 import SMBus # Ensure you have run: pip install smbus2

# Define the robot
romi = AStar()

# --- NEW: I2C Gyro Constants ---
LSM6_ADDR = 0x6b
CTRL2_G = 0x11
OUTZ_L = 0x26
bus = SMBus(1)

# Initialize the Gyro (Set to 1.66 kHz, 2000 dps scale)
bus.write_byte_data(LSM6_ADDR, CTRL2_G, 0x8C)

def get_gyro_z_raw():
    """Reads the raw Z-axis value from the IMU chip directly."""
    try:
        data = bus.read_i2c_block_data(LSM6_ADDR, OUTZ_L, 2)
        raw_z = data[0] | (data[1] << 8)
        if raw_z > 32767:
            raw_z -= 65536
        return raw_z
    except:
        return 0

# --- Your Constants ---
WHEEL_DIAMETER_MM = 70.0
COUNTS_PER_REV = 1440
WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * math.pi
MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV

# Scale for 2000 dps is usually 0.070 for LSM6 chips
GYRO_SCALE = 0.070 

def calibrate_gyro(samples=100):
    print("Calibrating gyro... do not move the robot.")
    total = 0
    for i in range(samples):
        total += get_gyro_z_raw()
        time.sleep(0.005)
    print("Calibration done.")
    return total / samples

GYRO_BIAS = calibrate_gyro()

def go_straight(robot, distance_mm, base_speed=100, kP=2.0):
    target_counts = distance_mm / MM_PER_COUNT
    robot.reset_encoders()

    heading = 0
    last_time = time.time()

    while True:
        # Note: In standard a_star.py, encoders() returns a list
        encoders = robot.read_encoders() 
        left, right = encoders[0], encoders[1]
        avg = (abs(left) + abs(right)) / 2

        if avg >= target_counts:
            break

        # Update Heading
        gz = get_gyro_z_raw()
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        angular_velocity = (gz - GYRO_BIAS) * GYRO_SCALE
        heading += angular_velocity * dt 

        correction = kP * heading
        
        left_speed = int(base_speed - correction)
        right_speed = int(base_speed + correction)

        robot.motors(left_speed, right_speed)

    robot.motors(0, 0)

def turn(robot, target_angle, speed=80):
    robot.motors(0,0)
    time.sleep(0.2)

    angle_turned = 0
    last_time = time.time()
    direction = 1 if target_angle > 0 else -1

    # Start rotating
    robot.motors(speed * direction, -speed * direction)

    while abs(angle_turned) < abs(target_angle):
        gz = get_gyro_z_raw()
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        angular_velocity = (gz - GYRO_BIAS) * GYRO_SCALE
        angle_turned += angular_velocity * dt
    
    robot.motors(0,0)

# Example usage:
# go_straight(romi, 500) # Go 50cm
# turn(romi, 90)         # Turn 90 degrees