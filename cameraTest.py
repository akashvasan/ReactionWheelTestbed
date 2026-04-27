"""
Camera test — saves a snapshot to disk for orientation check.
Run with: /usr/bin/python3 cameraTest.py
Output: camera_snapshot.jpg in the same directory
"""

import cv2
import time
from picamera2 import Picamera2

def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1280, 720)})
    picam2.configure(config)
    picam2.start()
    print("Camera warming up...")
    time.sleep(2)

    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    out = "/home/fri/Desktop/ReactionWheelTestbed/camera_snapshot.jpg"
    cv2.imwrite(out, frame)
    print(f"Snapshot saved to {out}")
    print("Open it on your laptop to check orientation.")

    picam2.stop()

if __name__ == "__main__":
    main()
