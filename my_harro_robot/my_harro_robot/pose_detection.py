#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class YoloPoseNode(Node):
    def __init__(self):
        super().__init__('yolo_pose_node')
        self.declare_parameter('image_topic', '/harro_camera_feed/image_raw')
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.model = YOLO("yolov8n-pose.pt")

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(Image, '/pose_img', 10)
        self.get_logger().info(f'Subscribed to {self.image_topic}, publishing pose image to /pose_img')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Run YOLO pose detection
        results = self.model.predict(source=frame, save=False, verbose=False)

        annotated_frame = results[0].plot()

        # Publish the processed image
        try:
            pose_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.pub.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(f"Image publishing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
