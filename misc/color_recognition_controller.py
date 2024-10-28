import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from PIL import Image as pImage

class ColorRecognitionNode(Node):
    def __init__(self):
        super().__init__('color_recognition')

        self.camera_feed_subscription = self.create_subscription(
            msg_type=Image,
            topic='camera/live_feed',
            callback=self.listener_callback,
            qos_profile=10
        )
        
        self.turn_dir_publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10
        )

        self.bridge = CvBridge()
        self.colors = {
            "green": [170, 193, 73],
            "blue": [173, 83, 0],
            "yellow": [103, 194, 255]
        }
        self.limits = self.get_limits()
        self.get_logger().info('Color Recognition Node initialized.')

    def listener_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert received img
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue_in_frame = False

        for key, value in self.limits.items():
            lower_limit_hsv, upper_limit_hsv = value
            mask = cv2.inRange(hsv_frame, lower_limit_hsv, upper_limit_hsv)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)

            mask_image = pImage.fromarray(mask)
            mask_bbox = mask_image.getbbox()

            if mask_bbox:
                x1, y1, x2, y2 = mask_bbox
                frame = cv2.rectangle(frame, pt1=(x1, y1), pt2=(x2, y2), color=self.colors[key], thickness=5)

                if key == 'blue':
                    bbox_center_x = x1 + (x2-x1)/2
                    blue_in_frame = True

                    turn_direction = Twist()
                    turn_direction.angular.z = 0.5 if bbox_center_x < mask_image.width/2 else -0.5

                    self.turn_dir_publisher.publish(turn_direction) 
                    
                    self.get_logger().info(f'Turning with angular velocity: {turn_direction.angular.z}')

        if not blue_in_frame:
            turn_direction = Twist()
            turn_direction.angular.z = 0.0
            self.turn_dir_publisher.publish(turn_direction)

        cv2.imshow('frame', frame)
        cv2.waitKey(1) # wait 1 ms for correct framerate and user inputs


    def get_limits(self):
        limits = {}

        for key, value in self.colors.items():
            color = np.uint8([[value]])
            hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            hue = hsv_color[0][0][0]

            lower_hue = hue - 10

            if hue < lower_hue:
                lower_hue = 0

            upper_hue = (hue + 10) %180

            lower_limit = np.array([lower_hue, 100, 100], dtype=np.uint8)
            upper_limit = np.array([upper_hue, 255, 255], dtype=np.uint8)

            limits[key] = [lower_limit, upper_limit]

        return limits


def main(args=None):
    rclpy.init(args=args)
    color_recognition_node = ColorRecognitionNode()
    rclpy.spin(color_recognition_node)

    color_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()