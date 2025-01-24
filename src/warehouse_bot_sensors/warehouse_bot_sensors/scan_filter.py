import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from dotenv import load_dotenv
import os

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        self.min_range = float(os.getenv('SCAN_MIN_RANGE', 1.5))

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # needed to get infos from /scan
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, # needed so that rviz can read it
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber für das ursprüngliche /scan/raw Topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sub
        )
        
        # Publisher für das gefilterte /scan Topic
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_pub)

    def scan_callback(self, msg):
        # filtered_ranges = list(msg.ranges)
        # for i in range(80, 280): 
        #     filtered_ranges[i] = float('inf')  # set to infinity -> no obstacle

        filtered_ranges = [
            r if r > self.min_range else float('inf') for r in msg.ranges
        ]

        msg.ranges = filtered_ranges
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    load_dotenv()
    scan_filter = ScanFilter()
    try:
        rclpy.spin(scan_filter)
    except KeyboardInterrupt:
        pass
    finally:
        scan_filter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
