#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ScanListener(Node):
    def __init__(self):
        super().__init__('scan_listener')

        # Anpassung der QoS-Einstellungen für Best Effort und Keep Last
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # nur die letzten 10 Nachrichten speichern
        )

        # Subscriber für das /scan Topic mit angepasstem QoS-Profil
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # verhindern, dass Python den Subscriber vorzeitig abräumt
        self.get_logger().info("ScanListener Node gestartet, wartet auf /scan-Daten.")

    def listener_callback(self, msg):
        # Ausgabe von Informationen zur Reichweite
        self.get_logger().info(f'Scan-Daten empfangen: {msg.ranges[:10]} ...')
        # Hier werden nur die ersten 10 Distanzwerte ausgegeben. 
        # Entferne [:10], um alle Werte anzuzeigen.

def main(args=None):
    rclpy.init(args=args)
    node = ScanListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
