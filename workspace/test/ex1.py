import cv2
from openpose import pyopenpose as op
import os
import sys


def main():
    net_width = 256

    params = dict()
    params["model_folder"] = os.environ["OPENPOSE_MODEL_PATH"]
    # params["face"] = True
    # params["hand"] = True
    params["net_resolution"] = f"-1x{net_width}"
    # params["model_pose"] = "COCO"
    params["model_pose"] = "BODY_25"

    try:
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()

        # webcam
        cap = cv2.VideoCapture(-1)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 1)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            datum = op.Datum()
            datum.cvInputData = frame
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))

            threshold_height = int(frame.shape[0] * 0.5)
            print(is_fallen(datum.poseKeypoints, threshold_height))

            cv2.imshow("Webcam Live", datum.cvOutputData)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()
    except Exception as e:
        print(e)
        sys.exit(-1)


def is_fallen(keypoints: op.Datum.poseKeypoints, threshold_height: int):
    if keypoints is None:
        return False

    for person in keypoints:
        flag = True
        for part in person:
            if part[0] == 0.0 and part[1] == 0.0:
                continue
            if part[1] < threshold_height:
                flag = False
                break
        if flag:
            return True
    return False


if __name__ == "__main__":
    main()
