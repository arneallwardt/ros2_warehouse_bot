import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import OptimizePose
from tf2_ros import Buffer, TransformListener
from dotenv import load_dotenv


class PoseOptimizer(Node):
    def __init__(self):
        super().__init__('pose_optimizer')

        self.get_logger().info('pose_optimizer initialized.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10)
        
        self._action_server = ActionServer(
            self,
            OptimizePose,
            'optimize_pose',
            execute_callback=self.optimize_pose_callback)


    def optimize_pose_callback(self, goal_handle):
        self.get_logger().info('def align_product_callback')
        self.goal_handle = goal_handle
        self.PRODUCT_NOT_SEEN_COUNT = self.PRODUCT_NOT_SEEN_COUNT_BASE

        success = self.optimize_pose(goal_handle)
            
        result = OptimizePose.Result()

        result.product_diameter = self.current_product_diameter
        result.product_center_offset = self.current_product_center_offset
        result.success = success

        return result
    
    def optimize_pose(goal_handle):
        pass


    def provide_feedback(self, goal_handle):
        feedback = OptimizePose.Feedback()

        feedback.product_diameter = self.current_product_diameter
        feedback.product_center_offset = self.current_product_center_offset

        goal_handle.publish_feedback(feedback)
    
    

def main(args=None):
    rclpy.init(args=args)
    load_dotenv()

    product_aligner = PoseOptimizer()
    product_aligner.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()