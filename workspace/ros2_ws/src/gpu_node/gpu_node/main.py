import rclpy
import os

from .gpu_node import GPUNode

def main(args=None):
    net_width = 256

    params = dict()
    params["model_folder"] = os.environ["OPENPOSE_MODEL_PATH"]
    # params["face"] = True
    # params["hand"] = True
    params["net_resolution"] = f"-1x{net_width}"
    params["model_pose"] = "BODY_25"

    
    try:
        rclpy.init(args=args)
        gpu_node = GPUNode(op_params=params)
        rclpy.spin(gpu_node)
    except KeyboardInterrupt:
        pass
    finally:
        gpu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
