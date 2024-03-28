import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GPUNode(Node):
    def __init__(self):
        super().__init__("gpu_node")
        self.create_subscription(String, "/pose_image", self.sub_callback, 10)

    def sub_callback(self, msg):
        self.get_logger().info("Received: {}".format(msg))


def main(args=None):
    rclpy.init(args=args)
    gpu_node = GPUNode()
    try:
        rclpy.spin(gpu_node)
    except KeyboardInterrupt:
        pass
    finally:
        gpu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
