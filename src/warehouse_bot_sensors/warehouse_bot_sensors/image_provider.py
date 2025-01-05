import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
from dotenv import load_dotenv

class ImageProvider(Node):
    def __init__(self):
        super().__init__('image_provider')
        self.publisher = self.create_publisher(
            msg_type=Image, 
            topic='camera_image_raw', 
            qos_profile=10) # profile which sets the connection quality
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.camera_idx = 0 # 0 = built in camera
        
        # initialize camera and set resolution
        self.get_logger().info('Initializing camera...')
        self.cap = cv2.VideoCapture(self.camera_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(os.getenv('CAP_WIDTH', 320)))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(os.getenv('CAP_HEIGHT', 320)))

        if not self.cap.isOpened():
            self.get_logger().error('image_provider failed to open camera.')
            return
        self.get_logger().info('image_provider successfully initialized!')

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_img)
        else:
            self.get_logger().warn('image_provider ailed to read frame from camera.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    load_dotenv()
    camera_feed_publisher = ImageProvider()

    if camera_feed_publisher.cap.isOpened():  # Proceed only if camera opened successfully
        try:
           rclpy.spin(camera_feed_publisher)
        except KeyboardInterrupt:
           camera_feed_publisher.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()