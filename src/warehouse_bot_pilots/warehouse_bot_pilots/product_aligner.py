import rclpy
from rclpy.node import Node

class ProductAligner(Node):
    def __init__(self):
        super().__init__('product_aligner')


        self.get_logger().info('product_info_provider initialized.')


def main(args=None):
    rclpy.init(args=args)
    product_aligner = ProductAligner()
    rclpy.spin(product_aligner)

    product_aligner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()