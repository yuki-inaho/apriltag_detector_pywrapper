from april_detector_pywrapper import PyAprilTagDetector
import numpy as np
import cv2
import pdb

video_mode = True

def main():
    if not video_mode:
        img = cv2.imread("/home/inaho-04/Project/apriltag_detector_pywrapper/example/1596950807490547895.png")
        height, width, _ = img.shape

        fx = 600
        fy = 600
        cx = int(width/2)
        cy = int(height/2)
        tagSize = 0.025

        detector = PyAprilTagDetector()
        detector.setImageSize(width, height)
        detector.setCameraParams(fx, fy, cx, cy)
        detector.setTagSize(tagSize)
        detector.setTagCodes("36h11".encode('utf-8'))
        detector.setup()
        detector.getDetectedInfo(img, draw_flag=False)
        taginfo_list = detector.extractTagInfo()
        res_img = detector.getDetectedImage()

        intrinsic_parameter_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        while cv2.waitKey(10) & 0xFF != 27:
            cv2.imshow("res_img", res_img)

        cv2.imwrite("res_img.png", res_img)
        cv2.waitKey(10)
        cv2.destroyAllWindows()
    else:
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
        while cv2.waitKey(10) & 0xFF != 27:
            for i in range(10): # for reflesh image buffer
                status, img = cam.read()
            if not status:
                continue
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            detector.getDetectedInfo(img, draw_flag=True)
            taginfo_list = detector.extractTagInfo()
            res_img = detector.getDetectedImage()
            cv2.imshow("res_img", res_img)
        
        cv2.imwrite("res_img.png", res_img)
        cv2.waitKey(10)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
