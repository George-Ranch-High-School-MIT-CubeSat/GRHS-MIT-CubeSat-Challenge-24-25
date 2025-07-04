import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
import subprocess
import os
from picamera2 import Picamera2

# VARIABLES
THRESHOLD = 0      # Any desired value from the accelerometer
REPO_PATH = "/home/CubeSat/GRHS-MIT-CubeSat-Challenge-24-25/FlatSat Challenge Repo"  # Your GitHub repo path
FOLDER_PATH = "Images"   # Your image folder path in your GitHub repo

# imu and camera initialization
i2c = board.I2C()
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
picam2 = Picamera2()


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


def main():
    take_photo()


if __name__ == '__main__':
    main()
