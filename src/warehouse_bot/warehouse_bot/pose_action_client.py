import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.current_goal_pose = 0

        self.goal_poses = [
            {'x': -1.418, 'y': 0.623, 'z': -0.126, 'w': 0.992},
            {'x': -0.897, 'y': 1.194, 'z': 0.2227371335029602, 'w': 0.974},
            {'x': -1.418, 'y': 0.623, 'z': -0.126, 'w': 0.992},
        ]

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
        else:
            self.get_logger().error('Error while traveling to the goal pose.')

        self.current_goal_pose += 1
        if self.current_goal_pose < len(self.goal_poses):
            next_pose = self.get_next_pose()
            self.send_goal(next_pose)
        else:
            self.get_logger().info('No more poses to visit')


    def feedback_callback(self, feedback_msg):
        # Ausgabe von Feedback während der Navigation
        self.get_logger().info(f'Current pose feedback: {feedback_msg.feedback.current_pose.pose}')


def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseClient()

    pose = node.get_next_pose()

    node.send_goal(pose)
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
