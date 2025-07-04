"""
The code you will write for this module should calculate
roll, pitch, and yaw (RPY) and calibrate your measurements
for better accuracy. Your functions are split into two activities.
The first is basic RPY from the accelerometer and magnetometer. The
second is RPY using the gyroscope. Finally, write the calibration functions.
Run plot.py to test your functions, this is important because auto_camera.py 
relies on your sensor functions here.
"""

#import libraries
import time
import numpy as np
import time
import os
import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL

#imu initialization
i2c = busio.I2C(board.SCL, board.SDA)
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)


#Activity 1: RPY based on accelerometer and magnetometer
def roll_am(accelX,accelY,accelZ):
    roll = (180/np.pi) * np.arctan2(accelY, (np.sqrt(np.power(accelX, 2) + np.power(accelZ, 2))))
    return roll

def pitch_am(accelX,accelY,accelZ):
    pitch = (180/np.pi) * np.arctan2(accelX, (np.sqrt(np.power(accelY, 2) + np.power(accelZ, 2))))
    return pitch

def yaw_am(accelX,accelY,accelZ,magX,magY,magZ):
    mag_x = magX * np.cos(pitch_am(accelX, accelY, accelZ)) + magY * np.sin(roll_am(accelX, accelY, accelZ)) * np.sin(pitch_am(accelX, accelY, accelZ)) + magZ * np.cos(roll_am(accelX, accelY, accelZ)) * np.sin(roll_am(accelX, accelY, accelZ))
    mag_y = magY * np.cos(roll_am(accelX, accelY, accelZ)) - magZ * np.sin(roll_am(accelX, accelY, accelZ))
    return (180/np.pi) * np.arctan2(-mag_y, mag_x)

#Activity 2: RPY based on gyroscope
def roll_gy(prev_angle, delT, gyro):
    roll = prev_angle + gyro * delT
    return roll
def pitch_gy(prev_angle, delT, gyro):
    pitch = prev_angle + gyro * delT
    return pitch
def yaw_gy(prev_angle, delT, gyro):
    yaw = prev_angle + gyro * delT
    return yaw

#Activity 3: Sensor calibration
def calibrate_mag():
    #TODO: Set up lists, time, etc
    mag_list = []
    print("Preparing to calibrate magnetometer. Please wave around.")
    time.sleep(3)
    print("Calibrating...")
    magX_max = -10e9
    magX_min = 10e9
    magY_max = -10e9
    magY_min = 10e9
    magZ_max = -10e9
    magZ_min = 10e9
    start_time = time.time()
    duration = 5
    while time.time() - start_time < duration:
        magX, magY, magZ = mag.magnetic #gauss
        if magX > magX_max:
            magX_max = magX
        if magX < magX_min:
            magX_min = magX

        if magY > magY_max:
            magY_max = magY
        if magY < magY_min:
            magY_min = magY

        if magZ > magZ_max:
            magZ_max = magZ
        if magZ < magZ_min:
            magZ_min = magZ
    #TODO: Calculate calibration constants
    mag_list.append(magX_max - magX_min)
    mag_list.append(magY_max - magY_min)
    mag_list.append(magZ_max - magZ_min)
    print("Calibration complete.")
    return mag_list

def calibrate_gyro():
    #TODO
    gyro_list = []
    print("Preparing to calibrate gyroscope. Put down the board and do not touch it.")
    time.sleep(3)
    print("Calibrating...")
    gyroX_max = -10e9
    gyroX_min = 10e9
    gyroY_max = -10e9
    gyroY_min = 10e9
    gyroZ_max = -10e9
    gyroZ_min = 10e9
    start_time = time.time()
    duration = 5
    while time.time() - start_time < duration:
        gyroX, gyroY, gyroZ = accel_gyro.gyro #rad/s
        if gyroX > gyroX_max:
            gyroX_max = gyroX
        if gyroX < gyroX_min:
            gyroX_min = gyroX

        if gyroY > gyroY_max:
            gyroY_max = gyroY
        if gyroY < gyroY_min:
            gyroY_min = gyroY

        if gyroZ > gyroZ_max:
            gyroZ_max = gyroZ
        if gyroZ < gyroZ_min:
            gyroZ_min = gyroZ
    #TODO
    gyro_list.append(gyroX_max - gyroX_min)
    gyro_list.append(gyroY_max - gyroY_min)
    gyro_list.append(gyroZ_max - gyroZ_min)
    print("Calibration complete.")
    return gyro_list

def set_initial(mag_offset = [0,0,0]):
    """
    This function is complete. Finds initial RPY values.

    Parameters:
        mag_offset (list): magnetometer calibration offsets
    """
    #Sets the initial position for plotting and gyro calculations.
    print("Preparing to set initial angle. Please hold the IMU still.")
    time.sleep(3)
    print("Setting angle...")
    accelX, accelY, accelZ = accel_gyro.acceleration #m/s^2
    magX, magY, magZ = mag.magnetic #gauss
    #Calibrate magnetometer readings. Defaults to zero until you
    #write the code
    magX = magX - mag_offset[0]
    magY = magY - mag_offset[1]
    magZ = magZ - mag_offset[2]
    roll = roll_am(accelX, accelY,accelZ)
    pitch = pitch_am(accelX,accelY,accelZ)
    yaw = yaw_am(accelX,accelY,accelZ,magX,magY,magZ)
    print("Initial angle set.")
    return [roll,pitch,yaw]
