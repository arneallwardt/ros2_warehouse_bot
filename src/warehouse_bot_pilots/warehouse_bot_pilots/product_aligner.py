import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from geometry_msgs.msg import Twist
from warehouse_bot_interfaces.msg import ProductInfo
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import AlignProduct
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ProductAligner(Node):
    def __init__(self):
        super().__init__('product_aligner')

        self.get_logger().info('product_info_provider initialized.')

        # keep track of theese using the subscription on /product_info
        self.current_product_distance = 0.0 # TODO: should be None instead
        self.current_product_diameter = 0.0
        self.current_product_center_offset = 0.0

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

        is_product_diameter_optimized = True
        is_product_center_offset_optimized = False
        is_product_distance_optimized = False


        while (not (is_product_diameter_optimized and is_product_center_offset_optimized and is_product_distance_optimized)):
            self.optimize_product_center_offset(goal_handle.request)
            self.optimize_product_distance(goal_handle.request)
            #self.optimize_product_diameter(goal_handle.request)

            is_product_diameter_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_diameter, 
                tolerance=goal_handle.request.product_diameter_tolerance, 
                actual=self.current_product_diameter)
            
            is_product_center_offset_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_center_offset, 
                tolerance=goal_handle.request.product_center_offset_tolerance, 
                actual=self.current_product_center_offset)
            
            is_product_distance_optimized = self.is_parameter_optimized(
                goal=goal_handle.request.product_distance, 
                tolerance=goal_handle.request.product_distance_tolerance, 
                actual=self.current_product_distance)

            self.provide_feedback(goal_handle)
            print(is_product_center_offset_optimized, is_product_diameter_optimized, is_product_distance_optimized)

        return True
    

    def optimize_product_diameter(self, request):
        
        goal = request.product_diameter
        tolerance = request.product_diameter_tolerance
        pass


    def optimize_product_center_offset(self, request):

        goal = request.product_center_offset
        tolerance = request.product_center_offset_tolerance
        
        while not self.is_parameter_optimized(goal, tolerance, self.current_product_center_offset):
            center_offset = self.current_product_center_offset # to prevent to get updated center_offset mid function
            direction = 1 if center_offset < 0 else -1
            scaling_factor = 0.5

            offset_normalized = abs(center_offset) / 160 # normalize offset between [0, 1]
            abs_turn_speed = float(scaling_factor * (offset_normalized ** 2))

            turn_direction_msg = Twist()
            turn_direction_msg.angular.z = self.clamp(value=abs_turn_speed, min_value=0.015, max_value=0.5) * direction
            self.cmd_vel_publisher.publish(turn_direction_msg)
            print(f'turn speed: {turn_direction_msg.angular.z}')

        stop_msg = Twist()
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)

        # use this to stop the rotation of the bot bot:
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'


    def optimize_product_distance(self, request):

        goal = request.product_distance
        tolerance = request.product_distance_tolerance
        
        while not self.is_parameter_optimized(goal, tolerance, self.current_product_distance):
            distance = self.current_product_distance # to prevent to get updated center_offset mid function

            scaling_factor = 0.15
            movement_speed = float(scaling_factor * (distance ** 2))

            movement_msg = Twist()
            movement_msg.linear.x = self.clamp(value=movement_speed, min_value=0.01, max_value=0.05) # exponential decrease in speed
            print(f'movement speed: {movement_msg.linear.x}')
            self.cmd_vel_publisher.publish(movement_msg)

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(stop_msg)


    def is_parameter_optimized(self, goal, tolerance, actual):
        print(f'goal: {goal}')
        print(f'tolerance: {tolerance}')
        print(f'actual: {actual}')
        is_optim = abs(goal-actual) < tolerance
        print(is_optim)
        return is_optim


    def provide_feedback(self, goal_handle):
        feedback = AlignProduct.Feedback()

        feedback.product_diameter = self.current_product_diameter
        feedback.product_center_offset = self.current_product_center_offset
        feedback.product_distance = self.current_product_distance

        goal_handle.publish_feedback(feedback)


    def product_info_callback(self, msg):
        # update product info
        self.current_product_distance = msg.product_distance
        # self.current_product_diameter = msg.product_diameter
        self.current_product_center_offset = msg.product_center_offset
        
    
    def clamp(self, value, min_value, max_value):
        # make sure that parameters are in a sensible range
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)

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