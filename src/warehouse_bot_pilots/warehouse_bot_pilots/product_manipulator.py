import rclpy
import rclpy.callback_groups
from open_manipulator_msgs.srv import SetJointPosition
from rclpy.node import Node
from transitions import Machine
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import ManipulateProduct
from dotenv import load_dotenv
import os
import time


class ProductManipulator(Node):
    def __init__(self):
        super().__init__('product_manipulator')
        self.is_holding_product = False
        self._arm_client = self.create_client(SetJointPosition, '/open_manipulator/goal_joint_space_path')
        self._gripper_client = self.create_client(SetJointPosition, '/open_manipulator/goal_tool_control')

        self._action_server = ActionServer(
            self,
            ManipulateProduct,
            'manipulate_product',
            execute_callback=self.manipulate_product_callback)
        
        # state machine implementation
        states = [
            'initial',
            'idle',
            'hover_over_product',
            'align_gripper_with_product',
            'grip_product',
            'lower_product',
            'release_product',
            'error'
        ]
        self.machine = Machine(model=self, states=states, initial='idle')

        # state transitions
        self.machine.add_transition(
            trigger='start_idle', 
            source=['idle', 'grip_product', 'release_product'], 
            dest='idle',
            after=self.move_arm_to_goal_pose
        ) 
        self.machine.add_transition(
            trigger='start_hovering_over_product', 
            source='idle', 
            dest='hover_over_product',
            after=self.move_arm_to_goal_pose
        )
        self.machine.add_transition(
            trigger='start_aligning_gripper_with_product', 
            source='hover_over_product', 
            dest='align_gripper_with_product',
            after=self.move_arm_to_goal_pose
        ) 
        self.machine.add_transition(
            trigger='start_gripping_product', 
            source='align_gripper_with_product', 
            dest='grip_product',
            after=self.adjust_gripper
        ) 
        self.machine.add_transition(
            trigger='start_lowering_product', 
            source='idle', 
            dest='lower_product',
            after=self.move_arm_to_goal_pose
        )
        self.machine.add_transition(
            trigger='start_releasing_product', 
            source='lower_product', 
            dest='release_product',
            after=self.adjust_gripper
        )
        self.machine.add_transition(
            trigger='error', 
            source='*', 
            dest='error'
        )
    

        self.get_logger().info('product_manipulator initialized.')


    def sendSetJointPositionRequest(self, client, joint_names, joint_positions):

        client.wait_for_service()
        self.get_logger().info('SetJointPosition Client available!')

        request = SetJointPosition.Request()
        request.joint_position.joint_name = joint_names
        request.joint_position.position = joint_positions
        request.path_time = float(os.getenv('PATH_TIME', 2.0))

        joint_pos_future = client.call_async(request)
        rclpy.spin_until_future_complete(self, joint_pos_future)

        if joint_pos_future.result():
            time.sleep(float(os.getenv('PATH_TIME', 2.0))) # wait until movement is over
            
            # remember if robot is currently holding a product
            if self.state == 'grip_product':
                self.is_holding_product = True
            elif self.state == 'release_product':
                self.is_holding_product = False

            self.get_logger().info('Openmanipulator reached its goal pose.')
        else:
            self.get_logger().error('Error while moving openmanipulator to goal pose.')

        
    def move_arm_to_goal_pose(self):

        if self.state == 'idle':
            joint_pos_label = 'IDLE'
        elif self.state == 'hover_over_product':
            joint_pos_label = 'HOVER'
        elif self.state == 'align_gripper_with_product':
            joint_pos_label = 'ALIGN'
        elif self.state == 'lower_product':
            joint_pos_label = 'LOWER'
        else: 
            self.get_logger().info(f'{self.state} is not a valid state for moving the arm. Entering error pose.')
            self.error()

        self.get_logger().info(f'Moving arm in state {self.state}')

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        joint_positions = [
            float(os.getenv(f'JOINT1_{joint_pos_label}', 0.0)),
            float(os.getenv(f'JOINT2_{joint_pos_label}', 0.0)), 
            float(os.getenv(f'JOINT3_{joint_pos_label}', 0.0)), 
            float(os.getenv(f'JOINT4_{joint_pos_label}', 0.0))
        ]

        self.sendSetJointPositionRequest(
            client=self._arm_client, 
            joint_names=joint_names,
            joint_positions=joint_positions
        )

        
    def adjust_gripper(self):
        if self.state == 'grip_product':
            joint_positions = [-0.01]
        elif self.state == 'release_product' or 'idle':
            joint_positions = [0.01]
        else: 
            self.get_logger().info(f'{self.state} is not a valid state for adjusting the gripper. Entering error pose.')
            self.error()
        
        self.get_logger().info(f'Moving gripper in state {self.state}')

        self.sendSetJointPositionRequest(
            client=self._gripper_client,
            joint_names=['gripper'],
            joint_positions=joint_positions
        )
        

    def manipulate_product_callback(self, goal_handle):
        self.get_logger().info(f'def grip_product_callback. task: {goal_handle.request.task}')

        try:
            # make sure to start in idle state
            if not self.state == 'idle':
                self.get_logger().info('Not in idle state, entering idle...')
                self.start_idle()

            if goal_handle.request.task == 'grip':
                self.get_logger().info('starting to grip product.')
                self.start_hovering_over_product()
                self.start_aligning_gripper_with_product()
                self.start_gripping_product()
                self.start_idle()
            elif goal_handle.request.task == 'release':
                self.start_lowering_product()
                self.start_releasing_product()
                self.start_idle()
                self.get_logger().info('starting to release product.')
                
            else:
                self.get_logger().info('No valid task for product_manipulator. Aborting action...')
                goal_handle.canceled()
                return ManipulateProduct.Result()
            
            success = True

        except Exception as e:
            self.get_logger().error(f'product_manipulator failed. Error: {e}')
            success = False

        goal_handle.succeed()
        result = ManipulateProduct.Result()
        result.is_holding_product = self.is_holding_product
        result.success = success
        return result


def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    product_manipulator = ProductManipulator()
    product_manipulator.start_idle()

    rclpy.spin(product_manipulator)
    rclpy.shutdown()    

if __name__ == '__main__':
    main()