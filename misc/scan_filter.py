import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        # QoS-Einstellungen, die zu TurtleBot3s /scan-Topic passen
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber für das ursprüngliche /scan/raw Topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan/raw',
            self.scan_callback,
            qos_profile
        )
        
        # Publisher für das gefilterte /scan Topic
        self.publisher = self.create_publisher(LaserScan, '/scan/filtered', qos_profile)

    def scan_callback(self, msg):
        # Beispiel-Filter: Entferne alle Werte zwischen 160 und 200 Grad
        filtered_ranges = list(msg.ranges)
        for i in range(160, 200):
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
