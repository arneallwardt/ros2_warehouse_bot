import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
from transitions import Machine
from warehouse_bot_interfaces.action import AlignProduct, ManipulateProduct, MoveBack 
from open_manipulator_msgs.srv import SetJointPosition
import os
from dotenv import load_dotenv

class WarehouseBotMain(Node):
    def __init__(self):
        super().__init__('warehouse_bot_main')
        self._navigate_to_pose_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._align_product_action_client = ActionClient(self, AlignProduct, 'align_product')
        self._manipulate_product_action_client = ActionClient(self, ManipulateProduct, 'manipulate_product')
        self._move_back_action_client = ActionClient(self, MoveBack, 'move_back')

        self._joint_postion_client = self.create_client(SetJointPosition, '/open_manipulator/goal_joint_space_path')
        # pose information
        self.current_goal_pose_idx = 0
        self.is_holding_product = False

        self.goal_poses = [
            {
                'x': float(os.getenv('GOAL_1_X', 0.0)), 
                'y': float(os.getenv('GOAL_1_Y', 0.0)), 
                'z': float(os.getenv('GOAL_1_Z', 0.0)), 
                'w': float(os.getenv('GOAL_1_W', 0.0)),
            },
            {
                'x': float(os.getenv('GOAL_2_X', 0.0)), 
                'y': float(os.getenv('GOAL_2_Y', 0.0)), 
                'z': float(os.getenv('GOAL_2_Z', 0.0)), 
                'w': float(os.getenv('GOAL_2_W', 0.0)),
            },
            {
                'x': float(os.getenv('GOAL_3_X', 0.0)), 
                'y': float(os.getenv('GOAL_3_Y', 0.0)), 
                'z': float(os.getenv('GOAL_3_Z', 0.0)),
                'w': float(os.getenv('GOAL_3_W', 0.0)),
            },
            {
                'x': float(os.getenv('HOME_X', 0.0)), 
                'y': float(os.getenv('HOME_Y', 0.0)), 
                'z': float(os.getenv('HOME_Z', 0.0)), 
                'w': float(os.getenv('HOME_W', 0.0)),
            },
        ]

        for pose in self.goal_poses:
            self.get_logger().info(f'x: {pose["x"]}')
            self.get_logger().info(f'y: {pose["y"]}')
            self.get_logger().info(f'z: {pose["z"]}')
            self.get_logger().info(f'w: {pose["w"]}')

        # state machine implementation
        states = [
            'idle',
            'navigating',
            'moving_back',
            'aligning_with_product',
            'grabbing_product',
            'putting_down_product',
            'error',
        ]
        self.machine = Machine(model=self, states=states, initial='idle')

        # state transitions
        self.machine.add_transition(
            trigger='start_idle', 
            source=['putting_down_product', 'navigating'], 
            dest='idle',
            after=self.idle) 
        
        self.machine.add_transition(
            trigger='start_navigation', 
            source=['idle', 'moving_back', 'aligning_with_product'], 
            dest='navigating',
            after=self.navigate_to_next_pose)
        
        self.machine.add_transition(
            trigger='start_moving_back', 
            source=['grabbing_product'],
            dest='moving_back',
            after=self.call_move_back_server)
        
        self.machine.add_transition(
            trigger='start_aligning_with_product', 
            source=['navigating'], 
            dest='aligning_with_product',
            after=self.call_product_aligner)
        
        self.machine.add_transition(
            trigger='start_grabbing_product', 
            source=['aligning_with_product'], 
            dest='grabbing_product',
            after=self.call_product_manipulator)
        
        self.machine.add_transition(
            trigger='start_putting_down_product', 
            source=['navigating'], 
            dest='putting_down_product',
            after=self.call_product_manipulator)
        
        self.machine.add_transition(
            trigger='error', 
            source='*', 
            dest='error',
            after=lambda: self.get_logger().error('Bot entered error state.'))
    
        

    ### ACTION SERVER CALLS ###    
    
    def call_move_back_server(self):
        if self.state != 'moving_back':
            self.get_logger().info(f"warehouse_bot_main: wrong state for call_move_back_server(). Current state: {self.state}")
            return
        
        self.get_logger().info(f'calling call_move_back_server in current state: {self.state}')
        self.send_move_back_goal()
    

    ### align_with_product
    def call_product_aligner(self):
        if self.state != 'aligning_with_product':
            self.get_logger().info(f"warehouse_bot_main: wrong state for call_product_aligner(). Current state: {self.state}")
            return
         
        self.get_logger().info(f'calling product_aligner in current state: {self.state}')
        self.send_align_product_goal()


    ### grip product
    def call_product_manipulator(self):
        if self.state == 'grabbing_product':
            task = 'grip'
        elif self.state == 'putting_down_product':
            task = 'release'
        else:
            self.get_logger().info(f"warehouse_bot_main: wrong state for call_product_gripper(). Current state: {self.state}")
            self.error() 
        
        self.get_logger().info(f'calling call_product_gripper in current state: {self.state}; task: {task}')
        self.send_manipulate_product_goal(task)



    ### NAVIGATE ACTION

    def navigate_to_next_pose(self):
        self.get_logger().info(f'Navigating to next pose. current pose index: {self.current_goal_pose_idx}')
        pose = self.get_current_goal_pose()
        self.get_logger().info(f'Retrieved next goal pose. current pose index: {self.current_goal_pose_idx}')        
        self.send_navigate_to_pose_goal(pose)


    def get_current_goal_pose(self):

        if self.current_goal_pose_idx >= len(self.goal_poses):
            self.get_logger().info('No more poses to visit')
            self.start_idle()

        # if bot is holding product, navigate to last possible pose which is final pose where product should be layed down
        if self.is_holding_product:
            self.get_logger().info('Holding product. Navigating back to start pose.')
            self.current_goal_pose_idx = len(self.goal_poses)-1

        # Erstelle eine Zielposition
        pose = PoseStamped()

        pose.header.frame_id = 'map'  
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.goal_poses[self.current_goal_pose_idx]['x']  
        pose.pose.position.y = self.goal_poses[self.current_goal_pose_idx]['y']  
        pose.pose.orientation.z = self.goal_poses[self.current_goal_pose_idx]['z']  # TODO: drop z since it is essentially not used
        pose.pose.orientation.w = self.goal_poses[self.current_goal_pose_idx]['w']  
        
        self.current_goal_pose_idx += 1

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

            if self.is_holding_product:
                self.start_putting_down_product()
            else:
                self.start_aligning_with_product()

        else:
            self.get_logger().error('Error while traveling to the goal pose.')
            self.error()


    def navigate_to_pose_feedback_callback(self, feedback_msg):
        # Ausgabe von Feedback während der Navigation
        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            self.get_logger().info(f'Current pose feedback: {feedback_msg.feedback.current_pose.pose}')



    ### ALIGN PRODUCT ACTION

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



    ### MANIPULATE PRODUCT ACTION

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
            self.is_holding_product = result.is_holding_product

            if result.is_holding_product:
                self.start_moving_back()
            else:
                self.start_idle()

        else:
            self.get_logger().error('Empty manipulate_product action result')
            self.error()

    
    def manipulate_product_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            self.get_logger().info('####### MANIPULATE PRODUCT FEEDBACK #########')
            self.get_logger().info(f'holding product: {feedback.is_holding_product}')

    

    ### MOVE BACK ACTION

    def send_move_back_goal(self):
        goal_msg = MoveBack.Goal()
        
        goal_msg.speed = float(os.getenv('MOVE_BACK_SPEED', 0.2))
        goal_msg.duration = float(os.getenv('MOVE_BACK_DURATION', 2))

        self._move_back_action_client.wait_for_server()

        self.move_back_send_goal_future = self._move_back_action_client.send_goal_async(goal_msg, feedback_callback=self.move_back_feedback_callback)

        self.move_back_send_goal_future.add_done_callback(self.move_back_goal_response_callback) 

    
    def move_back_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('MoveBack Goal rejected :(')
            return

        self.get_logger().info('MoveBack Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.move_back_result_callback)


    def move_back_result_callback(self, future):
        result = future.result().result

        if result is not None:
            self.get_logger().info('####### MOVE BACK RESULT #########')
            
            if result.success:
                self.get_logger().info('####### SUCCESS #########')
                self.start_navigation()
            else: 
                self.get_logger().info('####### ERROR #########')
                self.send_move_back_goal()

        else:
            self.get_logger().error('Empty manipulate_product action result')
            self.error()

    
    def move_back_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if os.getenv('LOG_ACTION_FEEDBACK', False) == "True":
            self.get_logger().info('####### MOVE BACK FEEDBACK #########')
            self.get_logger().info(f'time_remaining: {feedback.time_remaining}')


    ### Basic states

    def log_error(self):
        self.get_logger().error('Bot has entered error state.')

    def idle(self):
        self.get_logger().info('Bot has entered idle state.')


def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    time.sleep(1) # wait till other packages are ready

    warehouse_bot_main = WarehouseBotMain()
    warehouse_bot_main.start_navigation()    
    
    rclpy.spin(warehouse_bot_main)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
