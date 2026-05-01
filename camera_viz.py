"""
Camera detection pipeline visualizer
=====================================
Captures one frame with the same settings used in cameraTracking.py, then
saves five annotated images that illustrate every stage of the hot-pink
color-blob detection pipeline.

Output (written to the same directory):
  viz_1_raw.jpg          — rotated camera frame, no processing
  viz_2_hsv_mask.jpg     — binary mask: white = pixels inside HSV range
  viz_3_masked_color.jpg — original color visible only where mask is white
  viz_4_contours.jpg     — all detected contours drawn on the frame
  viz_5_annotated.jpg    — final result: bounding box, centroid, center line,
                           error value and state label

Run with: /usr/bin/python3 camera_viz.py
"""

import os
import time
import cv2
import numpy as np
from picamera2 import Picamera2

EXPOSURE_US   = 2000
ANALOGUE_GAIN = 4.0
FRAME_W, FRAME_H = 640, 480
CX = FRAME_W // 2

HSV_LOWER = np.array([140, 150, 150])
HSV_UPPER = np.array([170, 255, 255])
MIN_BLOB_AREA = 500

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

def save(name, img):
    path = os.path.join(OUT_DIR, name)
    cv2.imwrite(path, img)
    print(f"  Saved {name}")

def label(img, text, pos, color=(255, 255, 255), scale=0.55, thickness=1):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, text, (pos[0]+1, pos[1]+1), font, scale, (0,0,0), thickness+1, cv2.LINE_AA)
    cv2.putText(img, text, pos,                  font, scale, color,   thickness,   cv2.LINE_AA)

def main():
    print("Initialising camera…")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
    picam2.set_controls({
        "AeEnable":     False,
        "ExposureTime": EXPOSURE_US,
        "AnalogueGain": ANALOGUE_GAIN,
    })
    picam2.start()
    time.sleep(2)
    print("Camera ready. Capturing frame…\n")

    frame_rgb = picam2.capture_array()
    picam2.stop()

    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    raw = frame.copy()
    label(raw, "1) Raw camera frame", (10, 24), color=(200, 255, 200))
    save("viz_1_raw.jpg", raw)

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    label(mask_bgr, "2) HSV mask  (white = target range)", (10, 24), color=(180, 180, 255))
    label(mask_bgr, f"HSV lower: H={HSV_LOWER[0]} S={HSV_LOWER[1]} V={HSV_LOWER[2]}",
          (10, FRAME_H - 32), color=(200, 200, 200), scale=0.45)
    label(mask_bgr, f"HSV upper: H={HSV_UPPER[0]} S={HSV_UPPER[1]} V={HSV_UPPER[2]}",
          (10, FRAME_H - 14), color=(200, 200, 200), scale=0.45)
    save("viz_2_hsv_mask.jpg", mask_bgr)

    color_isolated = cv2.bitwise_and(frame, frame, mask=mask)
    label(color_isolated, "3) Color isolated by mask", (10, 24), color=(200, 255, 200))
    save("viz_3_masked_color.jpg", color_isolated)

    contour_img = frame.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)
    label(contour_img, f"4) Contours detected: {len(contours)}", (10, 24), color=(0, 255, 0))
    label(contour_img, f"Min blob area: {MIN_BLOB_AREA} px²",
          (10, FRAME_H - 14), color=(200, 200, 200), scale=0.45)
    save("viz_4_contours.jpg", contour_img)

    annotated = frame.copy()
    cv2.line(annotated, (CX, 0), (CX, FRAME_H), (255, 255, 0), 1)
    label(annotated, "center", (CX + 4, 18), color=(255, 255, 0), scale=0.45)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area >= MIN_BLOB_AREA:
            M = cv2.moments(largest)
            if M["m00"] > 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 200, 255), 2)
                label(annotated, f"area={int(area)} px²", (x, y - 8),
                      color=(0, 200, 255), scale=0.45)
                cv2.circle(annotated, (centroid_x, centroid_y), 6, (0, 0, 255), -1)
                cv2.circle(annotated, (centroid_x, centroid_y), 6, (255, 255, 255), 1)
                cv2.arrowedLine(annotated, (centroid_x, centroid_y),
                                (CX, centroid_y), (0, 255, 255), 2, tipLength=0.15)
                error = (centroid_x - CX) / CX
                state = "CENTERED" if abs(error) < 0.03 else "TRACKING"
                label(annotated,
                      f"centroid_x={centroid_x}  error={error:+.3f}  [{state}]",
                      (10, FRAME_H - 14), color=(0, 255, 255), scale=0.50)
        else:
            label(annotated,
                  f"Largest blob area={int(area)} px² < min {MIN_BLOB_AREA} — SEARCH",
                  (10, FRAME_H - 14), color=(0, 100, 255), scale=0.50)
    else:
        label(annotated, "No contours found — SEARCH", (10, FRAME_H - 14),
              color=(0, 100, 255), scale=0.50)

    label(annotated, "5) Final annotated frame", (10, 24), color=(200, 255, 200))
    save("viz_5_annotated.jpg", annotated)

    print("\nAll done. Open the viz_*.jpg files to see the detection pipeline.")

if __name__ == "__main__":
    main()
