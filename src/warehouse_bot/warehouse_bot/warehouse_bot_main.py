import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
from transitions import Machine
from geometry_msgs.msg import Twist
import math
from warehouse_bot_interfaces.msg import ProductInfo


class WarehouseBotMain(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.LOG_FEEDBACK = False

        self.product_info_subscribtion = self.create_subscription(
            ProductInfo,
            '/product_info',
            self.align_with_product,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10)

        # pose information
        self.current_goal_pose = 0
        self.goal_poses = [
            {'x': 1.45, 'y': 0.45, 'z': 0.0, 'w': 1.0},
            {'x': 1.45, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            {'x': 1.45, 'y': -0.45, 'z': 0.0, 'w': 1.0},
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
        ]

        # state machine implementation
        states = [
            'idle',
            'navigating',
            'detecting_product',
            'adjusting_position',
            'grabbing_product',
            'putting_down_product',
            'error',
            'universal'
        ]
        self.machine = Machine(model=self, states=states, initial='universal')

        # state transitions
        self.machine.add_transition(
            trigger='start_idle', 
            source=['putting_down_product', 'universal'], 
            dest='idle') 
        
        self.machine.add_transition(
            trigger='start_navigation', 
            source=['idle', 'detecting_product', 'grabbing_product', 'universal'], 
            dest='navigating',
            after=self.navigate_to_next_pose)
        
        self.machine.add_transition(
            trigger='start_detecting_product', 
            source=['navigating', 'adjusting_position', 'grabbing_product', 'universal'], 
            dest='detecting_product',
            after=self.wait_for_next_pose)
        
        self.machine.add_transition(
            trigger='start_adjusting_position', 
            source=['detecting_product', 'universal'], 
            dest='adjusting_position')
        
        self.machine.add_transition(
            trigger='start_grabbing_product', 
            source=['adjusting_position', 'universal'], 
            dest='grabbing_product')
        
        self.machine.add_transition(
            trigger='start_putting_down_product', 
            source=['navigating' , 'universal'], 
            dest='putting_down_product')
        
        self.machine.add_transition(
            trigger='error', 
            source='*', 
            dest='error',
            after=lambda: print('Error state'))
            
        self.machine.add_transition(
            trigger='start_universal', 
            source='*', 
            dest='universal') 
        
    
    ### adjusting_position

    def align_with_product(self, msg):
        if self.state != 'adjusting_position':
            return 
        print('Aligning with product')
        print(msg)

        direction = 1 if msg.center_offset < 0 else -1
        scaling_factor = 1

        offset_normalized = abs(msg.center_offset) / 160 # normalize offset between [0, 1]

        turn_direction = Twist()
        turn_direction.angular.z = scaling_factor * (offset_normalized ** 2) * direction
        print(turn_direction.angular.z)
        self.cmd_vel_publisher.publish(turn_direction)

        # use this to stop the bot:
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'


    ### navigating

    def wait_for_next_pose(self):
        print(f'Waiting. Current state: {self.state}')
        time.sleep(3)

        self.current_goal_pose += 1
        if self.current_goal_pose < len(self.goal_poses):
            self.start_navigation()
        else:
            self.get_logger().info('No more poses to visit')
            self.start_idle()

    def navigate_to_next_pose(self):
        self.get_logger().info('Navigating to next pose')
        pose = self.get_next_pose()
        print(f'Navigating to pose: {pose}')
        self.send_goal(pose)


    def get_next_pose(self):

        # Erstelle eine Zielposition
        pose = PoseStamped()

        pose.header.frame_id = 'map'  # Das Koordinatensystem
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.goal_poses[self.current_goal_pose]['x']  # Ziel-X-Koordinate
        pose.pose.position.y = self.goal_poses[self.current_goal_pose]['y']  # Ziel-Y-Koordinate
        pose.pose.orientation.z = self.goal_poses[self.current_goal_pose]['z']  # Rotation um die Z-Achse
        pose.pose.orientation.w = self.goal_poses[self.current_goal_pose]['w']  # Keine Drehung (quaternion)

        return pose


    ### navigate_to_pose action

    def send_goal(self, pose):
    
        self._action_client.wait_for_server()

        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # get Future object and register callback on it
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        # check if goal was accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal not accepted.')
            return

        self.get_logger().info('Goal accepted!')

        # add callback to run when action is finished
        result_future = goal_handle.get_result_async() # Future object for result of the action
        result_future.add_done_callback(self.result_callback) # add callback on future object


    def result_callback(self, future):
        result = future.result().result
        if result is not None:
            self.get_logger().info('Bot has reached the goal pose!')
            self.start_detecting_product()
        else:
            self.get_logger().error('Error while traveling to the goal pose.')
            self.error()


    def feedback_callback(self, feedback_msg):
        # Ausgabe von Feedback während der Navigation
        if self.LOG_FEEDBACK:
            self.get_logger().info(f'Current pose feedback: {feedback_msg.feedback.current_pose.pose}')


    ### Error

    def log_error(self):
        self.get_logger().error('Error state')

def main(args=None):
    rclpy.init(args=args)

    warehouse_bot_main = WarehouseBotMain()
    # warehouse_bot_main.start_navigation() TODO: uncomment when starting state is navigating
    warehouse_bot_main.start_adjusting_position()
    
    rclpy.spin(warehouse_bot_main)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
