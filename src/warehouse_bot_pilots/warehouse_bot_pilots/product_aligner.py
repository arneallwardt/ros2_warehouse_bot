import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from geometry_msgs.msg import Twist
from warehouse_bot_interfaces.msg import ProductInfo
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import AlignProduct
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import os
from dotenv import load_dotenv


class ProductAligner(Node):
    def __init__(self):
        super().__init__('product_aligner')

        self.get_logger().info('product_aligner initialized.')

        # keep track of theese using the subscription on /product_info
        self.current_product_diameter = 0.0
        self.current_product_center_offset = 0.0
        self.is_product_in_frame = False
        self.FEEDBACK_ITERATIONS = 100
        self.PRODUCT_NOT_SEEN_COUNT_BASE = 20
        self.PRODUCT_NOT_SEEN_COUNT = self.PRODUCT_NOT_SEEN_COUNT_BASE
        self.CAN_CANCEL_ACTION = False
        self.turn_direction_angular_z = 0.0
        self.turn_direction_linear_x = 0.0

        self.callback_group = ReentrantCallbackGroup()

        self.product_info_subscribtion = self.create_subscription(
            ProductInfo,
            'product_info',
            self.product_info_callback,
            10,
            callback_group=self.callback_group)
        
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10)
        
        self._action_server = ActionServer(
            self,
            AlignProduct,
            'align_product',
            execute_callback=self.align_product_callback,
            callback_group=self.callback_group)


    def align_product_callback(self, goal_handle):
        self.get_logger().info('def align_product_callback')
        self.reset_state()

        success = self.align_with_product(goal_handle)
            
        goal_handle.succeed()
        result = AlignProduct.Result()

        result.product_diameter = self.current_product_diameter
        result.product_center_offset = self.current_product_center_offset
        result.success = success

        return result


    def align_with_product(self, goal_handle):

        is_product_diameter_optimized = False
        is_product_center_offset_optimized = False
        feedback_iterations = self.FEEDBACK_ITERATIONS

        while (not (is_product_diameter_optimized and is_product_center_offset_optimized)):

            if self.CAN_CANCEL_ACTION:
                return False

            if self.is_product_in_frame: # only optmize if product in frame or else it will use float('inf') values
                self.optimize_product_center_offset()
                self.optimize_product_diameter(goal_handle.request)
            else:
                self.stop_movement()

            is_product_center_offset_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_center_offset, 
                tolerance=goal_handle.request.product_center_offset_tolerance, 
                actual=self.current_product_center_offset)
            
            is_product_diameter_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_diameter, 
                tolerance=goal_handle.request.product_diameter_tolerance, 
                actual=self.current_product_diameter)

            feedback_iterations -= 1
            if feedback_iterations <= 0:
                self.provide_feedback(goal_handle)
                feedback_iterations = self.FEEDBACK_ITERATIONS

        self.stop_movement()

        return True
    
    def stop_movement(self):
        stop_msg = Twist()
        stop_msg.angular.z = 0.0
        stop_msg.linear.x = 0.0

        self.turn_direction_linear_x = 0.0
        self.turn_direction_angular_z = 0.0

        self.cmd_vel_publisher.publish(stop_msg)
    

    def optimize_product_diameter(self, request):
        
        goal = request.product_diameter
        
        diameter_error = self.current_product_diameter - goal 

        direction = 1 if diameter_error < 0 else -1
        scaling_factor = 0.5

        diameter_error_normalized = abs(diameter_error) / int(os.getenv('CAP_WIDTH', 320)) # normalize offset between [0, 1]
        abs_movement_speed = float(scaling_factor * (diameter_error_normalized ** 2))

        movement_msg = Twist()
        movement_msg.linear.x = self.clamp(value=abs_movement_speed, min_value=0.01, max_value=0.1) * direction # exponential decrease in speed
        movement_msg.angular.z = self.turn_direction_angular_z # ensure that other turn direction is not reset
        
        self.turn_direction_linear_x = movement_msg.linear.x 
        
        self.cmd_vel_publisher.publish(movement_msg)


    def optimize_product_center_offset(self):
        
        center_offset = self.current_product_center_offset # to prevent to get updated center_offset mid function
        direction = 1 if center_offset < 0 else -1
        scaling_factor = 0.5

        offset_normalized = abs(center_offset) / 160 # normalize offset between [0, 1]
        abs_turn_speed = float(scaling_factor * (offset_normalized ** 2))

        turn_direction_msg = Twist()
        turn_direction_msg.angular.z = self.clamp(value=abs_turn_speed, min_value=0.02, max_value=0.8) * direction
        turn_direction_msg.linear.x = self.turn_direction_linear_x # ensure that other turn direction is not reset
        
        self.turn_direction_angular_z = turn_direction_msg.angular.z
        
        self.cmd_vel_publisher.publish(turn_direction_msg)


    def is_parameter_optimized(self, goal, tolerance, actual):
        return abs(goal-actual) < tolerance


    def provide_feedback(self, goal_handle):
        feedback = AlignProduct.Feedback()

        feedback.product_diameter = self.current_product_diameter
        feedback.product_center_offset = self.current_product_center_offset

        goal_handle.publish_feedback(feedback)


    def product_info_callback(self, msg):
        # update product info
        self.current_product_diameter = msg.product_diameter
        self.current_product_center_offset = msg.product_center_offset
        self.is_product_in_frame = msg.product_in_frame

        if not msg.product_in_frame:
            self.PRODUCT_NOT_SEEN_COUNT -= 1
            self.get_logger().info(f"product not seen count: {self.PRODUCT_NOT_SEEN_COUNT}")
            if self.PRODUCT_NOT_SEEN_COUNT <= 0:
                self.CAN_CANCEL_ACTION = True
        else:
            self.PRODUCT_NOT_SEEN_COUNT = self.PRODUCT_NOT_SEEN_COUNT_BASE
    
    
    def reset_state(self):
        self.current_product_diameter = 0.0
        self.current_product_center_offset = 0.0
        self.turn_direction_angular_z = 0.0
        self.turn_direction_linear_x = 0.0
        self.CAN_CANCEL_ACTION = False

        self.FEEDBACK_ITERATIONS = 100
        self.PRODUCT_NOT_SEEN_COUNT = self.PRODUCT_NOT_SEEN_COUNT_BASE

    
    # makes sure that parameters are in a sensible range
    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    executor = MultiThreadedExecutor()
    product_aligner = ProductAligner()
    executor.add_node(product_aligner)

    try:
        executor.spin()
    finally: 
        product_aligner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()