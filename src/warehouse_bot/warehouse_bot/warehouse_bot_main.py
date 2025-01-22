import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
from transitions import Machine
from warehouse_bot_interfaces.action import AlignProduct
from warehouse_bot_interfaces.action import ManipulateProduct
from open_manipulator_msgs.srv import SetJointPosition
import os
from dotenv import load_dotenv

class WarehouseBotMain(Node):
    def __init__(self):
        super().__init__('warehouse_bot_main')
        self._navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._align_product_action_client = ActionClient(self, AlignProduct, 'align_product')
        self._manipulate_product_action_client = ActionClient(self, ManipulateProduct, 'manipulate_product')

        self._joint_postion_client = self.create_client(SetJointPosition, '/open_manipulator/goal_joint_space_path')
        # pose information
        self.current_goal_pose = 0
        self.goal_poses = [
            {'x': 1.7, 'y': 0.45, 'z': 0.0, 'w': 1.0},
            {'x': 1.7, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            {'x': 1.7, 'y': -0.45, 'z': 0.0, 'w': 1.0},
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
            source=['putting_down_product', 'universal', 'aligning_with_product'], 
            dest='idle') 
        
        self.machine.add_transition(
            trigger='start_navigation', 
            source=['idle', 'detecting_product', 'aligning_with_product', 'grabbing_product', 'universal'], 
            dest='navigating',
            after=self.navigate_to_next_pose)
        
        self.machine.add_transition(
            trigger='start_detecting_product', 
            source=['navigating', 'aligning_with_product', 'grabbing_product', 'universal'], 
            dest='detecting_product',
            after=self.tmp_start_aligning)
        
        self.machine.add_transition(
            trigger='start_aligning_with_product', 
            source=['detecting_product', 'universal'], 
            dest='aligning_with_product',
            after=self.call_product_aligner)
        
        self.machine.add_transition(
            trigger='start_grabbing_product', 
            source=['aligning_with_product', 'universal'], 
            dest='grabbing_product',
            after=self.call_product_manipulator)
        
        self.machine.add_transition(
            trigger='start_putting_down_product', 
            source=['navigating' , 'universal'], 
            dest='putting_down_product',
            after=self.call_product_manipulator)
        
        self.machine.add_transition(
            trigger='error', 
            source='*', 
            dest='error',
            after=lambda: self.get_logger().error('Bot entered error state.'))
            
        self.machine.add_transition(
            trigger='start_universal', 
            source='*', 
            dest='universal') 
        

    def tmp_start_aligning(self): # TODO: remove
        self.start_aligning_with_product()
    
    ### grip product
    def call_product_manipulator(self):
        if self.state == 'grabbing_product':
            task = 'grip'
        elif self.state == 'putting_down_product':
            task = 'release'
        else:
            self.get_logger().info(f"warehouse_bot_main: wrong state for call_product_gripper(). Current state: {self.state}")
            return 
        
        self.get_logger().info(f'calling call_product_gripper in current state: {self.state}')
        self.send_manipulate_product_goal(task)
        

    ### align_with_product
    def call_product_aligner(self):
        if self.state != 'aligning_with_product':
            self.get_logger().info(f"warehouse_bot_main: wrong state for call_product_aligner(). Current state: {self.state}")
            return 
        
        self.get_logger().info(f'calling product_aligner in current state: {self.state}')
        self.send_align_product_goal()



    ### navigating

    def navigate_to_next_pose(self):
        self.get_logger().info('Navigating to next pose')
        pose = self.get_next_pose()
        self.send_navigate_to_pose_goal(pose)


    def get_next_pose(self):

        if self.current_goal_pose >= len(self.goal_poses):
            self.get_logger().info('No more poses to visit')
            self.start_idle()

        # Erstelle eine Zielposition
        pose = PoseStamped()

        pose.header.frame_id = 'map'  # Das Koordinatensystem
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.goal_poses[self.current_goal_pose]['x']  # Ziel-X-Koordinate
        pose.pose.position.y = self.goal_poses[self.current_goal_pose]['y']  # Ziel-Y-Koordinate
        pose.pose.orientation.z = self.goal_poses[self.current_goal_pose]['z']  # Rotation um die Z-Achse
        pose.pose.orientation.w = self.goal_poses[self.current_goal_pose]['w']  # Keine Drehung (quaternion)

        self.current_goal_pose += 1

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
        goal_msg.product_center_offset = float(os.getenv('PRODUCT_CENTER_OFFSET', 0.0))
        goal_msg.product_center_offset_tolerance = float(os.getenv('PRODUCT_CENTER_OFFSET_TOLERANCE', 10.0))

        self.get_logger().info('waiting for align product action server')
        self._align_product_action_client.wait_for_server()

        self.get_logger().info('Sendign align product goal')
        self.align_product_send_goal_future = self._align_product_action_client.send_goal_async(goal_msg, feedback_callback=self.align_product_feedback_callback)

        self.align_product_send_goal_future.add_done_callback(self.align_product_goal_response_callback) 

    
    def align_product_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('AlignProduct Goal rejected :(')
            self.error()
            return

        self.get_logger().info('AlignProduct Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.align_product_result_callback)


    def align_product_result_callback(self, future):
        result = future.result().result

        if result is not None:
            if result.success:
                self.get_logger().info('####### ALIGN PRODUCT RESULT #########')
                self.get_logger().info(f'diameter: {result.product_diameter}')
                self.get_logger().info(f'center offset: {result.product_center_offset}')
                self.start_grabbing_product()
            else:
                self.get_logger().info('####### ALIGN PRODUCT NOT SUCCESSFULL ########')
                self.start_navigation()

        else:
            self.get_logger().error('####### ERROR IN ALIGN PRODUCT ########')
            self.error()

    
    def align_product_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            self.get_logger().info('####### ALIGN PRODUCT FEEDBACK #########')
            self.get_logger().info(f'diameter: {feedback.product_diameter}')
            self.get_logger().info(f'center offset: {feedback.product_center_offset}')


    ### manipulate_product action 

    def send_manipulate_product_goal(self, task):
        goal_msg = ManipulateProduct.Goal()
        
        goal_msg.task = task

        self._manipulate_product_action_client.wait_for_server()

        self.manipulate_product_send_goal_future = self._manipulate_product_action_client.send_goal_async(goal_msg, feedback_callback=self.manipulate_product_feedback_callback)

        self.manipulate_product_send_goal_future.add_done_callback(self.manipulate_product_goal_response_callback) 

    
    def manipulate_product_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('ManipulateProduct Goal rejected :(')
            return

        self.get_logger().info('ManipulateProduct Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.manipulate_product_result_callback)


    def manipulate_product_result_callback(self, future):
        result = future.result().result

        if result is not None:
            self.get_logger().info('####### MANIPULATE PRODUCT RESULT #########')
            self.get_logger().info(f'holding product: {result.is_holding_product}')
            # self.start_idle()

        else:
            self.get_logger().error('Empty manipulate_product action result')
            self.error()

    
    def manipulate_product_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            self.get_logger().info('####### MANIPULATE PRODUCT FEEDBACK #########')
            self.get_logger().info(f'holding product: {feedback.is_holding_product}')


    ### Error

    def log_error(self):
        self.get_logger().error('Error state')


def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    time.sleep(1) # wait till other packages are ready

    warehouse_bot_main = WarehouseBotMain()
    warehouse_bot_main.start_navigation()
    # warehouse_bot_main.start_aligning_with_product()
    
    rclpy.spin(warehouse_bot_main)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
