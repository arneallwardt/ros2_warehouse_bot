import rclpy
import rclpy.callback_groups
from open_manipulator_msgs.srv import SetJointPosition
from rclpy.node import Node
from transitions import Machine
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import GripProduct
from dotenv import load_dotenv
import os
import time


class ProductGripper(Node):
    def __init__(self):
        super().__init__('product_gripper')
        self.holding_product = False
        self._joint_postion_client = self.create_client(SetJointPosition, '/open_manipulator/goal_joint_space_path')
        
        self._action_server = ActionServer(
            self,
            GripProduct,
            'grip_product',
            execute_callback=self.grip_product_callback)
        
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
        self.machine = Machine(model=self, states=states, initial='initial')

        # state transitions
        self.machine.add_transition(
            trigger='start_idle', 
            source=['initial', 'grip_product', 'release_product'], 
            dest='idle',
            after=self.move_arm_to_idle_pose
        ) 
        self.machine.add_transition(
            trigger='start_hovering_over_product', 
            source='idle', 
            dest='hover_over_product'
        )
        self.machine.add_transition(
            trigger='start_aligning_gripper_with_product', 
            source='hover_over_product', 
            dest='align_gripper_with_product'
        ) 
        self.machine.add_transition(
            trigger='start_gripping_product', 
            source='align_gripper_with_product', 
            dest='grip_product'
        ) 
        self.machine.add_transition(
            trigger='start_lowering_product', 
            source='idle', 
            dest='lower_product'
        )
        self.machine.add_transition(
            trigger='start_releasing_product', 
            source='lower_product', 
            dest='release_product'
        )
        self.machine.add_transition(
            trigger='error', 
            source='*', 
            dest='error',
            after=self.cancel_action())

        self.get_logger().info('product_gripper initialized.')
        
    def move_arm_to_pose(self, joint_positions, necessary_state):
        if self.state != necessary_state: 
            self.get_logger().info(f"[product_gripper]: wrong state for moving arm to pose. Necessary state: {necessary_state}, current state: {self.state}" ) 
            self.error()

        self._joint_postion_client.wait_for_service()
        self.get_logger().info('SetJointPosition Client available!')

        request = SetJointPosition.Request()
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = joint_positions
        request.path_time = float(os.getenv('PATH_TIME', 2.0))

        joint_pos_future = self._joint_postion_client.call_async(request)
        rclpy.spin_until_future_complete(self, joint_pos_future)

        if joint_pos_future.result():
            time.sleep(float(os.getenv('PATH_TIME', 2.0))) # wait until movement is over
            self.get_logger().info('Openmanipulator reached its goal pose.')
        else:
            self.get_logger().error('Error while moving openmanipulator to goal pose.')
        

    def move_arm_to_idle_pose(self):
        joint_positions = [
            float(os.getenv('JOINT1_IDLE', 0.0)),
            float(os.getenv('JOINT2_IDLE', 0.0)), 
            float(os.getenv('JOINT3_IDLE', 0.0)), 
            float(os.getenv('JOINT4_IDLE', 0.0))
        ]
        self.move_arm_to_pose(joint_positions, 'idle')


    def move_arm_to_hover_over_product(self):
        joint_positions = [
            float(os.getenv('JOINT1_HOVER', 0.0)),
            float(os.getenv('JOINT2_HOVER', 0.0)), 
            float(os.getenv('JOINT3_HOVER', 0.0)), 
            float(os.getenv('JOINT4_HOVER', 0.0))
        ]
        self.move_arm_to_pose(joint_positions, 'idle')
        

    def grip_product(self):
        if not self.state == 'idle':
            self.get_logger().info('Not in idle state, entering idle...')
            self.start_idle()
        self.start_hovering_over_product()
        self.start_aligning_gripper_with_product()
        self.start_gripping_product()


    def grip_product_callback(self, goal_handle):
        self.get_logger().info('def grip_product_callback')

        success = self.grip_product(goal_handle)

        if success:
            result = GripProduct.Result()
            goal_handle.succeed()
            
        # TODO: set result attributes

        return result


    def grip_product(self, goal_handle):
        # TODO: implementation
        pass


    def provide_feedback(self, goal_handle):
        feedback = GripProduct.Feedback()

        # TODO: set feedback attributes

        goal_handle.publish_feedback(feedback)

    def cancel_action(self):
         self.get_logger().info('product_gripper entered exit state. Cancelling Action.')


def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    product_gripper = ProductGripper()
    product_gripper.start_idle()
    print('PRODUCT GRIPPER INSTANTIATED')

    rclpy.spin(product_gripper)
    rclpy.shutdown()    

if __name__ == '__main__':
    main()