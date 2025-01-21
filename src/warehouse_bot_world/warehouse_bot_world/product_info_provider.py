import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from warehouse_bot_interfaces.msg import ProductInfo 
import os
from dotenv import load_dotenv

class ProductInfoProvider(Node):
    def __init__(self):
        super().__init__('product_info_provider')

        # create subscriber
        self.camera_feed_subscription = self.create_subscription(
            msg_type=Image,
            topic='camera_image_raw',
            callback=self.camera_image_raw_callback,
            qos_profile=10
        )

        # create publisher
        self.info_publisher = self.create_publisher(ProductInfo, 'product_info', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_product_info)
        self.product_in_frame = False
        self.center_offset = float('inf')
        self.product_diameter = float('inf')

        self.bridge = CvBridge()
        self.colors = {
            "blue": [76, 34, 7],
        }
        self.limits = self.get_limits()


        self.get_logger().info('product_info_provider initialized.')


    def publish_product_info(self):
        msg = ProductInfo()                                        
        msg.product_in_frame = self.product_in_frame
        msg.product_center_offset = self.center_offset
        msg.product_diameter = self.product_diameter
        self.info_publisher.publish(msg)


    def camera_image_raw_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert received img
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # check every color provided
        for key, value in self.limits.items():
            lower_limit_hsv, upper_limit_hsv = value
            mask = cv2.inRange(hsv_frame, lower_limit_hsv, upper_limit_hsv)

            # kernel = np.ones((5, 5), np.uint8)
            # mask = cv2.erode(mask, kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            if contours and key == 'blue':
                largest_contour = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                cv2.circle(frame, (int(x), int(y)), int(radius), self.colors[key], 3)

                self.product_in_frame = True 
                self.center_offset = x - int(os.getenv('CAP_WIDTH', 320))/2
                self.product_diameter = radius*2
            else:
                self.product_in_frame = False
                self.center_offset = float('inf')
                self.product_diameter = float('inf')

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
    load_dotenv()

    product_info_provider = ProductInfoProvider()
    rclpy.spin(product_info_provider)

    product_info_provider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()