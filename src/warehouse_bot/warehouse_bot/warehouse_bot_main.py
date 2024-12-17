import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
from transitions import Machine


class WarehouseBotMain(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # state machine implementation
        states = [
            'idle',
            'navigating',
            'detecting_product',
            'adjusting_position',
            'grabbing_product',
            'putting_down_product',
            'error'
        ]
        self.machine = Machine(model=self, states=states, initial='idle')

        # state transitions
        self.machine.add_transition(trigger='start_idle', source='putting_down_product', dest='idle')
        self.machine.add_transition(trigger='start_navigation', source=['idle', 'detecting_product', 'grabbing_product'], dest='navigating')
        self.machine.add_transition(trigger='start_detecting_product', source=['navigating', 'adjusting_position', 'grabbing_product'], dest='detecting_product')
        self.machine.add_transition(trigger='start_adjusting_position', source='detecting_product', dest='adjusting_position')
        self.machine.add_transition(trigger='start_grabbing_product', source='adjusting_position', dest='grabbing_product')
        self.machine.add_transition(trigger='start_detecting_product', source='navigating', dest='putting_down_product')
        self.machine.add_transition(trigger='start_idle', source='putting_down_product', dest='idle')
        self.machine.add_transition(trigger='error', source='*', dest='error')

        # state callbacks
        self.machine.on_enter_navigating(self.navigate_to_next_pose)
        self.machine.on_enter_idle(self.wait())
        self.machine.on_enter_error(lambda: print('Error state'))

        # pose information
        self.current_goal_pose = 0
        self.goal_poses = [
            {'x': 1.614, 'y': 0.464, 'z': 0.0, 'w': 1.0},
            {'x': 1.4335, 'y': -0.1385, 'z': 0.0, 'w': 0.9939},
            {'x': 1.7028, 'y': -0.3628, 'z': 0.0, 'w': 0.9931},
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
        ]

    def wait(self):
        print(f'Waiting. Current state: {self.state}')
        time.sleep(3)

        self.current_goal_pose += 1
        if self.current_goal_pose < len(self.goal_poses):
            self.start_navigation()
        else:
            self.get_logger().info('No more poses to visit')
            self.error()

    def navigate_to_next_pose(self):
        self.get_logger().info('Navigating to next pose')
        pose = self.get_next_pose()
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

    def log_error(self):
        self.get_logger().error('Error state')

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
            self.start_idle()
        else:
            self.get_logger().error('Error while traveling to the goal pose.')
            self.error()


    def feedback_callback(self, feedback_msg):
        # Ausgabe von Feedback während der Navigation
        self.get_logger().info(f'Current pose feedback: {feedback_msg.feedback.current_pose.pose}')


def main(args=None):
    rclpy.init(args=args)
    warehouse_bot_main = WarehouseBotMain()

    # switch to navigation state
    warehouse_bot_main.start_navigation()
    
    rclpy.spin(warehouse_bot_main)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
