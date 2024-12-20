import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as pImage
from warehouse_bot_interfaces.msg import ProductInfo # this works but somehow a warning is shown

class ProductInfoProvider(Node):
    def __init__(self):
        super().__init__('product_info_provider')

        # create subscriber
        self.camera_feed_subscription = self.create_subscription(
            msg_type=Image,
            topic='camera_image_raw',
            callback=self.listener_callback,
            qos_profile=10
        )

        # create publisher
        self.info_publisher = self.create_publisher(ProductInfo, 'product_info', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_product_info)
        self.product_in_frame = False
        self.center_offset = 0.0


        self.bridge = CvBridge()
        self.colors = {
            "blue": [173, 83, 0],
        }
        self.limits = self.get_limits()


        self.get_logger().info('product_info_provider initialized.')


    def publish_product_info(self):
        msg = ProductInfo()                                        
        msg.product_in_frame = self.product_in_frame
        msg.center_offset = self.center_offset
        self.info_publisher.publish(msg)


    def listener_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert received img
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for key, value in self.limits.items():
            lower_limit_hsv, upper_limit_hsv = value
            mask = cv2.inRange(hsv_frame, lower_limit_hsv, upper_limit_hsv)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)

            mask_image = pImage.fromarray(mask)
            mask_bbox = mask_image.getbbox()

            if mask_bbox and key == 'blue':

                # get bounding box
                x1, y1, x2, y2 = mask_bbox
                frame = cv2.rectangle(frame, pt1=(x1, y1), pt2=(x2, y2), color=self.colors[key], thickness=5)

                # center of bbox
                bbox_center = x1+(x2-x1)/2
                self.center_offset = bbox_center - mask_image.width/2
                self.product_in_frame = True 
            else:
                self.product_in_frame = False
                self.center_offset = 0.0

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
    product_info_provider = ProductInfoProvider()
    rclpy.spin(product_info_provider)

    product_info_provider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()