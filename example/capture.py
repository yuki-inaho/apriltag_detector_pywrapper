import os
from april_detector_pywrapper import PyAprilTagDetector
import numpy as np
import cv2
import pdb
import time

from pytz import timezone
from datetime import datetime

video_mode = True
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def get_time():
    utc_now = datetime.now(timezone('UTC'))
    jst_now = utc_now.astimezone(timezone('Asia/Tokyo'))
    time = str(jst_now).split(".")[0].split(" ")[
        0] + "_" + str(jst_now).split(".")[0].split(" ")[1]
    return time


def main():
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    status, img = cam.read()
    while not status:
        status, img = cam.read()

    height, width, _ = img.shape
    fx = 600.0
    fy = 600.0
    cx = float(int(width/2))
    cy = float(int(height/2))
    tagSize = 0.025

    detector = PyAprilTagDetector()
    detector.setImageSize(width, height)
    detector.setCameraParams(fx, fy, cx, cy)
    detector.setTagSize(tagSize)
    detector.setTagCodes("36h11".encode('utf-8'))
    detector.setup()

    intrinsic_parameter_matrix = np.array(
        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    key = cv2.waitKey(10)
    while key & 0xFF != 27:
        for i in range(10): # for reflesh image buffer
            status, img = cam.read()
        if not status:
            continue

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        detector.getDetectedInfo(img, draw_flag=True)
        taginfo_list = detector.extractTagInfo()
        res_img = detector.getDetectedImage()
        cv2.imshow("res_img", res_img)
        if key & 0xFF == ord('s'):
            time = get_time()
            cv2.imwrite("{}/{}.png".format(SCRIPT_DIR, time), img)
        key = cv2.waitKey(10)
    cv2.imwrite("res_img.png", res_img)
    cv2.waitKey(10)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
