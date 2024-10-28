import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from datetime import datetime

class CameraFeedPublisher(Node):
    def __init__(self):
        super().__init__('camera_feed_publisher')
        self.publisher = self.create_publisher(
            msg_type=Image, 
            topic='camera/live_feed', 
            qos_profile=10) # profile which sets the connection quality
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        
        # initialize camera and set resolution to 640x480
        self.get_logger().info('Initializing camera...')
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            return
        self.get_logger().info('Publisher Node successfully initialized!')

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_img)
            self.get_logger().info(f'Publishing Image at {datetime.now()}')
        else:
            self.get_logger().warn('Failed to read frame from camera.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_feed_publisher = CameraFeedPublisher()

    if camera_feed_publisher.cap.isOpened():  # Proceed only if camera opened successfully
        try:
           rclpy.spin(camera_feed_publisher)
        except KeyboardInterrupt:
           camera_feed_publisher.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
alias nb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias ltb='ros2 launch turtlebot3_bringup robot.l>
alias lst='ros2 launch slam_toolbox online_async_>
alias ln2='ros2 launch nav2_bringup navigation_la>
'''