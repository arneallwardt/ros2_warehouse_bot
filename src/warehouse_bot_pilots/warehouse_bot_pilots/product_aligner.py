import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from warehouse_bot_interfaces.msg import ProductInfo
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import AlignProduct
import random
import time


class ProductAligner(Node):
    def __init__(self):
        super().__init__('product_aligner')

        self.get_logger().info('product_info_provider initialized.')

        # keep track of theese using the subscription on /product_info
        self.current_product_distance = 0 # TODO: should be None instead
        self.current_product_diameter = 0
        self.current_product_center_offset = 0

        self.product_info_subscribtion = self.create_subscription(
            ProductInfo,
            '/product_info',
            self.product_info_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10)
        
        self._action_server = ActionServer(
            self,
            AlignProduct,
            'align_product',
            self.align_product_callback)


    def align_product_callback(self, goal_handle):
        self.get_logger().info('Aligning with product...')

        success = self.align_with_product(goal_handle.request)

        goal_handle.succeed()
        
        result = AlignProduct.Result()
        feedback = AlignProduct.Feedback()

        for i in range(5):
            self.current_product_center_offset += random.random()
            self.current_product_diameter += random.random()
            self.current_product_distance += random.random()

            feedback.product_center_offset = self.current_product_center_offset
            feedback.product_diameter = self.current_product_diameter
            feedback.product_distance = self.current_product_distance

            goal_handle.publish_feedback(feedback)

            time.sleep(1)
            

        result.product_diameter = self.current_product_diameter
        result.product_distance = self.current_product_distance
        result.product_center_offset = self.current_product_center_offset

        return result


    def align_with_product(self, request):
        print(request)
        return True


    def product_info_callback(self, msg):
        # update product info
        self.current_product_distance = msg.product_distance
        self.current_product_diameter = msg.product_diameter
        self.current_product_center_offset = msg.product_center_offset

    # def align_with_product(self, msg):
        
    #     print('Aligning with product')
    #     print(msg)

    #     direction = 1 if msg.center_offset < 0 else -1
    #     scaling_factor = 1

    #     offset_normalized = abs(msg.center_offset) / 160 # normalize offset between [0, 1]

    #     turn_direction = Twist()
    #     turn_direction.angular.z = scaling_factor * (offset_normalized ** 2) * direction
    #     print(turn_direction.angular.z)
    #     self.cmd_vel_publisher.publish(turn_direction)

        # use this to stop the bot:
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'



def main(args=None):
    rclpy.init(args=args)
    product_aligner = ProductAligner()
    rclpy.spin(product_aligner)

    product_aligner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()