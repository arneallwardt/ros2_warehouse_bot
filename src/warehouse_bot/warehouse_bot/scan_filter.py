import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

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
        self.publisher = self.create_publisher(LaserScan, '/scan/filtered', qos_profile_pub)

    def scan_callback(self, msg):
        # Beispiel-Filter: Entferne alle Werte zwischen 160 und 200 Grad
        filtered_ranges = list(msg.ranges)
        for i in range(90, 270): # 0 starts on the left of the bot and continues clockwise
            filtered_ranges[i] = float('inf')  # Setze auf "unendlich" (kein Hindernis)

        msg.ranges = filtered_ranges
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
