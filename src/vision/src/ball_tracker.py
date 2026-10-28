import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ultralytics import YOLO
import cv2
import numpy as np

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        
        self.model = YOLO('src/vision/src/best.pt')

        cv2.namedWindow("BINgo Ball Tracker", cv2.WINDOW_NORMAL)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/color_image/compressed',
            self.image_callback,
            qos_profile)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model(frame, verbose=False)

        annotated_frame = results[0].plot()

        cv2.imshow("BINgo Ball Tracker", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallTracker()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()