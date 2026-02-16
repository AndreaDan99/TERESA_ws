import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DebugDepth(Node):
    def __init__(self):
        super().__init__("debug_depth")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.cb,
            10,
        )

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.get_logger().info(f"Depth SHAPE: {img.shape}, dtype: {img.dtype}")
            self.get_logger().info(f"Sample pixel[240,320]: {img[240,320]}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DebugDepth()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
