import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
from transitions import Machine
from warehouse_bot_interfaces.action import AlignProduct
import os
from dotenv import load_dotenv

class WarehouseBotMain(Node):
    def __init__(self):
        super().__init__('warehouse_bot_main')
        self._navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._align_product_action_client = ActionClient(self, AlignProduct, 'align_product')
        self.LOG_FEEDBACK = True

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
            'aligning_with_product',
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
            source=['navigating', 'aligning_with_product', 'grabbing_product', 'universal'], 
            dest='detecting_product',
            after=self.wait_for_next_pose)
        
        self.machine.add_transition(
            trigger='start_aligning_with_product', 
            source=['detecting_product', 'universal'], 
            dest='aligning_with_product',
            after=self.call_product_aligner)
        
        self.machine.add_transition(
            trigger='start_grabbing_product', 
            source=['aligning_with_product', 'universal'], 
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
        

    ### align_with_product
    def call_product_aligner(self):
        if self.state != 'aligning_with_product':
            print(f"warehouse_bot_main: wrong state for call_product_aligner(). Current state: {self.state}")
            return 
        
        print(f'calling product_aligner in current state: {self.state}')
        self.send_align_product_goal()



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
        self.send_navigate_to_pose_goal(pose)


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

    def send_navigate_to_pose_goal(self, pose):
    
        self._navigate_to_pose_action_client.wait_for_server()

        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # get Future object and register callback on it
        future = self._navigate_to_pose_action_client.send_goal_async(goal_msg, feedback_callback=self.navigate_to_pose_feedback_callback)
        future.add_done_callback(self.navigate_to_pose_goal_response_callback)


    def navigate_to_pose_goal_response_callback(self, future):
        # check if goal was accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal not accepted.')
            return

        self.get_logger().info('Goal accepted!')

        # add callback to run when action is finished
        result_future = goal_handle.get_result_async() # Future object for result of the action
        result_future.add_done_callback(self.navigate_to_pose_result_callback) # add callback on future object


    def navigate_to_pose_result_callback(self, future):
        result = future.result().result
        if result is not None:
            self.get_logger().info('Bot has reached the goal pose!')
            self.start_detecting_product()
        else:
            self.get_logger().error('Error while traveling to the goal pose.')
            self.error()


    def navigate_to_pose_feedback_callback(self, feedback_msg):
        # Ausgabe von Feedback während der Navigation
        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            self.get_logger().info(f'Current pose feedback: {feedback_msg.feedback.current_pose.pose}')


    ### align_product action

    def send_align_product_goal(self):
        goal_msg = AlignProduct.Goal()
        
        goal_msg.product_diameter = float(os.getenv('PRODUCT_DIAMETER', 0.0))
        goal_msg.product_diameter_tolerance = float(os.getenv('PRODUCT_DIAMETER_TOLERANCE', 200.0))
        goal_msg.product_distance = float(os.getenv('PRODUCT_DISTANCE', 0.15))
        goal_msg.product_distance_tolerance = float(os.getenv('PRODUCT_DISTANCE_TOLERANCE', 0.01))
        goal_msg.product_center_offset = float(os.getenv('PRODUCT_CENTER_OFFSET', 0.0))
        goal_msg.product_center_offset_tolerance = float(os.getenv('PRODUCT_CENTER_OFFSET_TOLERANCE', 10.0))

        self._align_product_action_client.wait_for_server()

        self.align_product_send_goal_future = self._align_product_action_client.send_goal_async(goal_msg, feedback_callback=self.align_product_feedback_callback)

        self.align_product_send_goal_future.add_done_callback(self.align_product_goal_response_callback) 

    
    def align_product_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('AlignProduct Goal rejected :(')
            return

        self.get_logger().info('AlignProduct Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.align_product_result_callback)


    def align_product_result_callback(self, future):
        result = future.result().result

        if result is not None:
            print('####### ALIGN PRODUCT RESULT #########')
            print(f'diameter: {result.product_diameter}')
            print(f'distance: {result.product_distance}')
            print(f'center offset: {result.product_center_offset}')
            self.start_idle()

        else:
            self.get_logger().error('Empty align_product action result')
            self.error()

    
    def align_product_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            print('####### ALIGN PRODUCT FEEDBACK #########')
            print(f'diameter: {feedback.product_diameter}')
            print(f'distance: {feedback.product_distance}')
            print(f'center offset: {feedback.product_center_offset}')


    ### Error

    def log_error(self):
        self.get_logger().error('Error state')


def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    warehouse_bot_main = WarehouseBotMain()
    # warehouse_bot_main.start_navigation() TODO: uncomment when starting state is navigating
    warehouse_bot_main.start_aligning_with_product()
    
    rclpy.spin(warehouse_bot_main)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
