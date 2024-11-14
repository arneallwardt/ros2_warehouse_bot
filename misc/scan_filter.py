import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/scan/filtered', 10)

    def scan_callback(self, msg):
        # Beispiel: Entferne alle Werte zwischen 160 und 200 Grad
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
