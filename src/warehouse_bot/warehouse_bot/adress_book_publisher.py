import rclpy
from rclpy.node import Node

from more_interfaces.msg import AddressBook


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('adress_book_publisher')
        self.publisher_ = self.create_publisher(AddressBook, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AddressBook()
        msg.first_name = 'Arne'
        msg.last_name = 'Arnold'
        msg.phone_number = '123412341234'
        msg.phone_type = msg.PHONE_TYPE_WORK
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing {msg.first_name}, {msg.last_name}, {msg.phone_number}, {msg.phone_type}.')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
