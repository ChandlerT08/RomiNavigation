# Import Libraries
from a_star import AStar
import time
import math

# Define the robot
romi = AStar()

# Constants
WHEEL_DIAMETER_MM = 70.0
COUNTS_PER_REV = 1440

WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * math.pi
MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV

GYRO_SCALE = 0.00875 # in degrees / second per unit

# Calibrate the Gyro bias by averaging what the gyrometer reads when it is not rotating multiple times
def calibrate_gyro(robot, samples=100):
    total = 0
    for i in range(samples):
        gx, gy, gz = robot.gyro() #returns the gyro in the x, y, and z direction (only the z is needed though)
        total += gz
        time.sleep(0.005)
    return total / samples

GYRO_BIAS = calibrate_gyro(romi)

# Function to go straight
def go_straight(robot, distance_mm, base_speed=100, kP=2.0):
    
    # Establish what the target number of counts is
    target_counts = distance_mm / MM_PER_COUNT

    # reset the count of the encoders from previous movements
    robot.reset_encoders()

    # establish variables for gyro correction (uses the integration of the product of the angular velocity measured by the gyro and the delta time)
    heading = 0
    last_time = time.time()

    #actually power the motors until the target distance has been reached
    while True:
        left, right = robot.encoders()
        avg = (left + right) / 2

        if avg >= target_counts:
            break

        # update the heading from the gyro
        gx, gy, gz = robot.gyro()
        current_time = time.time()
        dt = current_time - last_time
        last_time - current_time

        angular_velocity = (gz - GYRO_BIAS) * GYRO_SCALE
        heading += angular_velocity * dt # Which adds the degrees we are off by

        # Correct the motor speeds to fix drift by multiplying the heading by our kP (proportional Gain value)
        correction = kP * heading
        
        #set motor speeds and power motor
        left_speed = int(base_speed - correction)
        right_speed = int(base_speed + correction)

        robot.motors(left_speed, right_speed)

    # zero the motors when target distance is met
    robot.motors(0,0)

# Function that turns using the gyrometer
def turn(robot, target_angle, speed=80):
    # zero the motors and wait for a small time to ensure no movement is occuring when the turning starts
    robot.motor(0,0)
    time.sleep(0.1)

    # instantiate the angle turned and time variable
    angle_turned = 0
    last_time = time.time()

    # set the direction we are turing and start turning
    direction = 1 if target_angle > 0 else -1
    robot.motors(speed * direction, -speed * direction)

    # measrue the gyro to figure out when to stop
    while abs(angle_turned) < abs(target_angle):
        # instantiate gyro
        gx, gy, gz = robot.gyro

        # find the dt
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # calculate angular velocity and find add to the angle turned
        angular_velocity = (gz - GYRO_BIAS) * GYRO_SCALE
        angle_turned += angular_velocity * dt
    
    # zero motors to end motion
    robot.motors(0,0)


# Special push function to ensure careful waterbottle relocation high kP for aggressive correction
def push_bottle(robot, distance_mm, base_speed=70, kP=3.0):
    #find the target counts to measure how for to travel
    target_counts = distance_mm / MM_PER_COUNT
    
    # reset the encoders
    robot.reset_encoders()

    # instantiate heading and time variable for gyro correction
    heading = 0
    last_time = time.time()

    while True:
        # find count and average them together and then break out of the loop if you have travelled the desired distance
        left, right = robot.encoders()
        avg = (left + right) / 2

        if avg >= target_counts:
            break

        # update the gyro
        gx, gy, gz = robot.gyro()

        # find the dt
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # find angular velocity and update heading
        angular_velocity = (gz - GYRO_BIAS) * GYRO_SCALE
        heading += angular_velocity * dt

        # calculate correction and then set motor speeds and power the motor
        correction = kP * heading

        left_speed = int(base_speed - correction)
        right_speed = int(base_speed + correction)

        robot.motors(left_speed, right_speed)

    # zero motors to signal end of motion
    robot.motors(0,0)