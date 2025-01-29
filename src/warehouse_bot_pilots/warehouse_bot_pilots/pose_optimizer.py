import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import OptimizePose
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
import math


class PoseOptimizer(Node):
    def __init__(self):
        super().__init__('pose_optimizer')

        self.get_logger().info('pose_optimizer initialized.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.FEEDBACK_RATE_DEFAULT = 400
        self.FEEDBACK_RATE = self.FEEDBACK_RATE_DEFAULT
        
        self._cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10)
        
        self._action_server = ActionServer(
            self,
            OptimizePose,
            'optimize_pose',
            execute_callback=self.optimize_pose_callback)
        
    
    def get_current_pose(self):
        try: 
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                rclpy.time.Time()) # get map -> base_footprint which is essentially current pose of bot

            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().error(f'Pose Optimizer: failed to get current pose: {str(e)}')
            return None


    def optimize_pose_callback(self, goal_handle):
        self.get_logger().info('def optimize_pose_callback')
        self.goal_handle = goal_handle

        result = self.optimize_pose(goal_handle)

        return result
    

    def optimize_pose(self, goal_handle):
        goal_pose = goal_handle.request.goal_pose
        xy_tolerance = goal_handle.request.xy_goal_tolerance
        yaw_tolerance = goal_handle.request.yaw_goal_tolerance

        while rclpy.ok():

            # get current pose information
            current_pose = self.get_current_pose()

            self.get_logger().info(f"current pose after method call: {current_pose}")

            if current_pose is None:
                continue

            current_x = current_pose.pose.position.x
            current_y = current_pose.pose.position.y
            current_orientation = self.get_yaw_from_quaternion(current_pose.pose.orientation)

            # get goal pose information
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            goal_orientation = self.get_yaw_from_quaternion(goal_pose.pose.orientation)

            # calculate current difference
            dx = goal_x - current_x
            dy = goal_y - current_y
            xy_error = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            yaw_error = angle_to_goal - current_orientation

            # check tolerances
            if xy_error <= xy_tolerance and abs(yaw_error) <= yaw_tolerance:
                self.get_logger().info('Pose optimization complete!')
                self.stop_movement()
                goal_handle.succeed()
                return OptimizePose.Result(success=True)
            
            self.publish_feedback(
                goal_handle=goal_handle,
                xy_error=yaw_error,
                yaw_error=yaw_error,
                current_x=current_x,
                current_y=current_y,
                current_w=current_pose.pose.orientation.w
            )
            
            # move to goal pose
            twist = Twist()
            twist.linear.x = self.clamp(value=(0.5 * xy_error), min_value=0.01, max_value=0.05)
            twist.angular.z = self.clamp(value=(1.0 * yaw_error), min_value=0.01, max_value=0.05)
            self._cmd_vel_publisher.publish(twist)

    
    def stop_movement(self):
        stop_msg = Twist()
        stop_msg.angular.z = 0.0
        stop_msg.linear.x = 0.0

        self.turn_direction_linear_x = 0.0
        self.turn_direction_angular_z = 0.0

        self._cmd_vel_publisher.publish(stop_msg)
            

    def get_yaw_from_quaternion(self, quaternion):
        qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    

    def publish_feedback(self, goal_handle, xy_error, yaw_error, current_x, current_y, current_w):

        self.FEEDBACK_RATE -= 1

        if self.FEEDBACK_RATE <= 0:

            current_pose = PoseStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.pose.position.x = current_x
            current_pose.pose.position.y = current_y 
            current_pose.pose.orientation.w = current_w 
            
            feedback = OptimizePose.Feedback()
            feedback.current_pose = current_pose
            feedback.current_xy_error = xy_error
            feedback.current_yaw_error = yaw_error

            goal_handle.publish_feedback(feedback)

            self.FEEDBACK_RATE = self.FEEDBACK_RATE_DEFAULT
    
    
    # makes sure that parameters are in a sensible range
    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)
    
    

def main(args=None):
    rclpy.init(args=args)

    product_aligner = PoseOptimizer()
    
    rclpy.spin(product_aligner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()