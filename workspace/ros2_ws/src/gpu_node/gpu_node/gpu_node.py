from rclpy.node import Node
from std_msgs.msg import String
from openpose import pyopenpose as op
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class GPUNode(Node):
    """
    OpenPose によって姿勢推定を行うノード。
    画像を受け取り、姿勢推定を行い、結果をトピックに送信する。
    """

    def __init__(
        self, camera_topic="/camera", pose_topic="/pose", op_params: dict = None
    ):
        """
        ## Parameters

        - camera_topic: カメラ画像を受け取るトピック名
        - pose_topic: 姿勢推定結果を送信するトピック名
        - op_params: OpenPose のパラメータ
        """

        super().__init__("gpu_node")
        # OpenPose を初期化
        self._opWrapper = op.WrapperPython()
        self._opWrapper.configure(op_params)
        self._opWrapper.start()

        self.create_subscription(Image, camera_topic, self._sub_camera_callback, 10)
        self._pose_publisher = self.create_publisher(String, pose_topic, 10)
        self._cv_bridge = CvBridge()

    def _sub_camera_callback(self, image: Image):
        """/camera トピックのコールバック関数"""

        self.get_logger().info("Received: /camera")

        frame = self._cv_bridge.imgmsg_to_cv2(image)
        key_points = self._proccess_image(frame)
        msg = String(data=str(key_points))

        self._pose_publisher.publish(msg)
        self.get_logger().info("Published: /pose")

    def _proccess_image(self, frame):
        """画像を処理し、姿勢推定結果を返す"""

        datum = op.Datum()
        datum.cvInputData = frame
        self._opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        if datum.poseKeypoints is None:
            return []

        return datum.poseKeypoints.tolist()
