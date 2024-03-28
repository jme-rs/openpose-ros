# https://github.com/spmallick/learnopencv/blob/master/OpenPose/OpenPoseImage.py

import cv2
import numpy as np
import os


def main():
    model_path = os.environ["OPENPOSE_MODEL_PATH"]
    proto_file_path = os.path.join(model_path, "pose/coco/pose_deploy_linevec.prototxt")
    weights_file_path = os.path.join(
        model_path, "pose/coco/pose_iter_440000.caffemodel"
    )
    n_points = 18
    POSE_PAIRS = [
        [1, 0],
        [1, 2],
        [1, 5],
        [2, 3],
        [3, 4],
        [5, 6],
        [6, 7],
        [1, 8],
        [8, 9],
        [9, 10],
        [1, 11],
        [11, 12],
        [12, 13],
        [0, 14],
        [0, 15],
        [14, 16],
        [15, 17],
    ]

    net = cv2.dnn.readNetFromCaffe(proto_file_path, weights_file_path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    cap = cv2.VideoCapture(-1)
    while cap.isOpened():
        has_frame, frame = cap.read()
        if not has_frame:
            break

        frame_width = frame.shape[1]
        frame_height = frame.shape[0]

        inpBlob = cv2.dnn.blobFromImage(
            frame,
            1.0 / 255,
            (frame_width, frame_width),
            (0, 0, 0),
            swapRB=False,
            crop=False,
        )
        net.setInput(inpBlob)
        output = net.forward()
        H = output.shape[2]
        W = output.shape[3]

        points = []

        for i in range(n_points):
            # confidence map of corresponding body's part.
            probMap = output[0, i, :, :]

            # Find global maxima of the probMap.
            minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)

            # Scale the point to fit on the original image
            x = (frame_width * point[0]) / W
            y = (frame_height * point[1]) / H

            if prob > 0.1:
                cv2.circle(
                    frame,
                    (int(x), int(y)),
                    8,
                    (0, 255, 255),
                    thickness=-1,
                    lineType=cv2.FILLED,
                )
                # cv2.putText(
                #     frame,
                #     "{}".format(i),
                #     (int(x), int(y)),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     1,
                #     (0, 0, 255),
                #     2,
                #     lineType=cv2.LINE_AA,
                # )

                # Add the point to the list if the probability is greater than the threshold
                points.append((int(x), int(y)))
            else:
                points.append(None)

        # Draw Skeleton
        for pair in POSE_PAIRS:
            partA = pair[0]
            partB = pair[1]

            if points[partA] and points[partB]:
                cv2.line(frame, points[partA], points[partB], (0, 255, 255), 2)
                cv2.circle(
                    frame,
                    points[partA],
                    8,
                    (0, 0, 255),
                    thickness=-1,
                    lineType=cv2.FILLED,
                )

        cv2.imshow("output", frame)
        if cv2.waitKey(100) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
