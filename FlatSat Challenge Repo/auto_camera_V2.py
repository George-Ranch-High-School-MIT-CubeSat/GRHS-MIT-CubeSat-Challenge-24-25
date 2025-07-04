"""
The code below is a template for the auto_camera.py file. You will need to
finish the capture() function to take a picture at a given RPY angle. Make
sure you have completed the sensor_calc.py file before you begin this one.
"""

#import libraries
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
import time
import os
import board
import busio
from picamera2 import Picamera2
import numpy as np
import sys
import subprocess
from sensor_calc_V2 import *
#from FlatSat_student import *

# VARIABLES
THRESHOLD = 0      # Any desired value from the accelerometer
REPO_PATH = "/home/CubeSat/GRHS-MIT-CubeSat-Challenge-24-25/FlatSat Challenge Repo"  # Your GitHub repo path
FOLDER_PATH = "Images"   # Your image folder path in your GitHub repo

#imu and camera initialization
i2c = busio.I2C(board.SCL, board.SDA)
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
picam2 = Picamera2()

#Code to take a picture at a given offset angle
def capture(dir ='roll', target_angle = 30):
    #Calibration lines should remain commented out until you implement calibration
    offset_mag = calibrate_mag()
    offset_gyro =calibrate_gyro()
    initial_angle = set_initial(offset_mag)
    prev_angle = initial_angle
    if dir = 'roll':
        numDir = 0
    elif dir = 'pitch':
        numDir = 1
    else {
        numDir = 2
    }
    print("Begin moving camera.")
    while True:
        accelX, accelY, accelZ = accel_gyro.acceleration #m/s^2
        magX, magY, magZ = mag.magnetic #gauss
	    #Calibrate magnetometer readings
        magX = magX - offset_mag[0]
        magY = magY - offset_mag[1]
        magZ = magZ - offset_mag[2]
        gyroX, gyroY, gyroZ = accel_gyro.gyro #rad/s
        #Convert to degrees and calibrate
        gyroX = gyroX *180/np.pi - offset_gyro[0]
        gyroY = gyroY *180/np.pi - offset_gyro[1]
        gyroZ = gyroZ *180/np.pi - offset_gyro[2]
        
        #TODO: Everything else! Be sure to not take a picture on exactly a
        #certain angle: give yourself some margin for error.
        while not(target_angle + 5 >= prev_angle[numDir] and target_angle - 5 <= prev_angle[numDir]):
            if prev_angle[numDir] > target_angle + 5:
                print("Move the CubeSat by at least " + str(prev_angle[numDir] - (target_angle + 5)) + " degrees to the left.")
            else:
                print("Move the CubeSat by at least " + str((target_angle + 5) - prev_angle[numDir]) + " degrees to the right.")
            prev_angle = set_initial(offset_mag)
        print("Hold position")
        take_photo()
        
#Code to take photos and store them
def git_push():
    """
    This function stages, commits, and pushes new images to your GitHub repo.
    Uses subprocess to interact with git commands directly.
    """
    try:
        print(f"Checking repo at {REPO_PATH}")
        
        # Check if the repo path exists and is correct
        if not os.path.isdir(REPO_PATH):
            print(f"ERROR: Repo path does not exist: {REPO_PATH}")
            return

        # Print the remote URL to verify
        print("Checking remote URL...")
        subprocess.run(["git", "-C", REPO_PATH, "remote", "get-url", "origin"], check=True)
        
        # Pull the latest changes from the remote repository
        print("Pulling latest changes from remote...")
        pull_process = subprocess.run(["git", "-C", REPO_PATH, "pull"], check=False, capture_output=True, text=True)
        
        if pull_process.returncode != 0:
            print(f"Error during git pull: {pull_process.stderr}")
        else:
            print("Pulled changes successfully.")
        
        # Stage and commit the changes using subprocess
        print("Staging files...")
        subprocess.run(["git", "-C", REPO_PATH, "add", "."], check=True)
        print("Files staged.")
        
        print("Committing changes...")
        subprocess.run(["git", "-C", REPO_PATH, "commit", "-m", "New Photo"], check=True)
        print("Commit successful.")
        
        # Push the changes to GitHub
        print("Pushing changes to GitHub...")
        subprocess.run(["git", "-C", REPO_PATH, "push"], check=True)
        print("Pushed changes successfully.")
    
    except subprocess.CalledProcessError as e:
        print(f"Error during git operation: {str(e)}")
    except Exception as e:
        print(f"Couldn't upload to git: {str(e)}")


def img_gen(name):
    """
    Generates a new image name with a timestamp.
    """
    t = time.strftime("_%H%M%S")
    imgname = f'{REPO_PATH}/{FOLDER_PATH}/{name}{t}.jpg'
    return imgname


def take_photo():
    """
    This function takes a photo when the FlatSat is shaken.
    """
    while True:
        accelx, accely, accelz = accel_gyro.acceleration

        # Check if the acceleration exceeds the threshold
        if accelx > THRESHOLD or accely > THRESHOLD or accelz > THRESHOLD:
            # Pause before taking a photo
            time.sleep(2)
            name = "EthanJ"  # Your name for the photo (e.g., EthanJ)
            
            # Take a photo with the camera
            picam2.start()
            picam2.capture_file(img_gen(name))
            picam2.stop()  # Ensure camera stops after taking the photo
            
            # Push the photo to GitHub
            git_push()

        # Pause to prevent constant checking
        time.sleep(2)
        

if __name__ == '__main__':
    capture(*sys.argv[1:])
