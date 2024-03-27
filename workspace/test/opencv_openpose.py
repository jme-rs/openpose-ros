import cv2
from openpose import pyopenpose as op
import os
import sys


def main():
    params = dict()
    params["model_folder"] = os.environ["OPENPOSE_MODEL_PATH"]
    params["face"] = True
    params["hand"] = True
    params["net_resolution"] = "-1x128"

    try:
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()

        # webcam
        cap = cv2.VideoCapture(-1)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            datum = op.Datum()
            datum.cvInputData = frame
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))

            cv2.imshow("Webcam Live", datum.cvOutputData)
            if cv2.waitKey(1000) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()
    except Exception as e:
        print(e)
        sys.exit(-1)


if __name__ == "__main__":
    main()
