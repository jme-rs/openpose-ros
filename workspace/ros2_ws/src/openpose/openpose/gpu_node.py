from rclpy.node import Node
from std_msgs.msg import String
from openpose import pyopenpose as op
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class GPUNode(Node):
    def __init__(self, camera_topic="/camera", pose_topic="/pose", op_params: dict = None):
        super().__init__("gpu_node")

        self.create_subscription(String, camera_topic, self._sub_camera_callback, 10)
        self._pose_publisher  = self.create_publisher(String, pose_topic, 10)
        self._cv_bridge = CvBridge()
        self._opWrapper = op.WrapperPython()
        
        self._opWrapper.configure(op_params)
        self._opWrapper.start()

    def _sub_camera_callback(self, image: Image):
        self.get_logger().info("Received: /camera")

        frame = self._cv_bridge.imgmsg_to_cv2(image)
        key_points = self._proccess_image(frame)
        msg = String(data=str(key_points))

        self._pose_publisher.publish(msg)
        self.get_logger().info("Published: /pose")

    def _proccess_image(self, frame):
        datum = op.Datum()
        datum.cvInputData = frame
        self._opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        return datum.poseKeypoints
    