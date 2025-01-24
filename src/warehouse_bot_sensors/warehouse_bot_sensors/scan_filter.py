import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from dotenv import load_dotenv
import os

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        self.min_range_front = float(os.getenv('SCAN_MIN_RANGE_FRONT', 0.1))
        self.min_range_back = float(os.getenv('SCAN_MIN_RANGE_BACK', 0.1))

        self.scan_back_start = int(os.getenv('SCAN_BACk_START', 90))
        self.scan_back_end = int(os.getenv('SCAN_BACk_END', 270))

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
        # filter all ranges for min_range_front (smaller)
        filtered_ranges = [
            r if r > self.min_range_front else float('inf') for r in msg.ranges
        ]

        # filter higher values for scans in the back to prevent hitting the open manipulator
        for i in range(self.scan_back_start, self.scan_back_end): 
            if filtered_ranges[i] < self.min_range_back:
                filtered_ranges[i] = float('inf')

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
