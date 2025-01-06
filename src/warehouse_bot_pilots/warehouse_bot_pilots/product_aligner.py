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
        self.current_product_distance = 0.0 # TODO: should be None instead
        self.current_product_diameter = 0.0
        self.current_product_center_offset = 0.0

        self.product_info_subscribtion = self.create_subscription(
            ProductInfo,
            'product_info',
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

        success = self.align_with_product(goal_handle)

        if success:
            result = AlignProduct.Result()
            goal_handle.succeed()
            

        result.product_diameter = self.current_product_diameter
        result.product_distance = self.current_product_distance
        result.product_center_offset = self.current_product_center_offset

        return result


    def align_with_product(self, goal_handle):

        print('def align_with_product...')

        is_product_diameter_optimized = False
        is_product_center_offset_optimized = False
        is_product_distance_optimized = False

        while (not (is_product_diameter_optimized and is_product_center_offset_optimized and is_product_distance_optimized)):
            self.optimize_product_diameter(goal_handle.request)
            self.optimize_product_distance(goal_handle.request)
            self.optimize_product_center_offset(goal_handle.request)

            is_product_diameter_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_diameter, 
                tolerance=goal_handle.request.product_diameter_tolerance, 
                actual=self.current_product_diameter)
            
            is_product_center_offset_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_distance, 
                tolerance=goal_handle.request.product_distance_tolerance, 
                actual=self.current_product_center_offset)
            
            is_product_distance_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_distance, 
                tolerance=goal_handle.request.product_distance_tolerance, 
                actual=self.current_product_distance)

            self.provide_feedback(goal_handle)

        return True
    

    def optimize_product_diameter(self, request):
        #print('optimzing diameter...')
        goal = request.product_diameter
        tolerance = request.product_diameter_tolerance

        while not self.is_parameter_optimized(goal, tolerance, self.current_product_distance):

            distance = self.current_product_distance # to prevent to get updated center_offset mid function

            scaling_factor = 1

            distance_capped = distance if distance < 0.5 else 0.5 # set max distance to 0.5 (to prevent adjusting too fast)

            movement_msg = Twist()
            movement_msg.linear.x = float(scaling_factor * (distance_capped ** 2)) # exponential decrease in speed
            self.cmd_vel_publisher.publish(movement_msg)

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(stop_msg)


    def optimize_product_center_offset(self, request):

        #print('optimzing center offset...')

        goal = request.product_center_offset
        tolerance = request.product_center_offset_tolerance
        
        while not self.is_parameter_optimized(goal, tolerance, self.current_product_center_offset):

            center_offset = self.current_product_center_offset # to prevent to get updated center_offset mid function

            direction = 1 if center_offset < 0 else -1
            scaling_factor = 1

            offset_normalized = abs(center_offset) / 160 # normalize offset between [0, 1]

            turn_direction_msg = Twist()
            turn_direction_msg.angular.z = float(scaling_factor * (offset_normalized ** 2) * direction)
            self.cmd_vel_publisher.publish(turn_direction_msg)
            # print(offset_normalized)
            # print(float(scaling_factor * (offset_normalized ** 2) * direction))

        stop_msg = Twist()
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)

        # use this to stop the rotation of the bot bot:
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'


    def optimize_product_distance(self, request):

        #print('optimzing distance...')

        goal = request.product_distance
        tolerance = request.product_distance_tolerance
        pass


    def is_parameter_optimized(self, goal, tolerance, actual):
        return abs(goal-actual) < tolerance


    def provide_feedback(self, goal_handle):
        feedback = AlignProduct.Feedback()

        feedback.product_diameter = self.current_product_diameter
        feedback.product_center_offset = self.current_product_center_offset
        feedback.product_distance = self.current_product_distance

        goal_handle.publish_feedback(feedback)


    def product_info_callback(self, msg):
        print(f'product_info_callback: {msg.product_center_offset}')
        # update product info
        self.current_product_distance = msg.product_distance
        # self.current_product_diameter = msg.product_diameter
        self.current_product_center_offset = msg.product_center_offset

    # def align_with_product(self, msg):
        
    
    def clamp(self, value, min_value, max_value):
        # make sure that parameters are in a sensible range
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    product_aligner = ProductAligner()
    rclpy.spin(product_aligner)

    product_aligner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()