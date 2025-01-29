import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from warehouse_bot_interfaces.action import MoveBack
import time


class MoveBackServer(Node):

    def __init__(self):
        super().__init__('move_back_server')

        self.get_logger().info('move_back_server initialized.')
        
        self._cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10)
        
        self._action_server = ActionServer(
            self,
            MoveBack,
            'move_back',
            execute_callback=self.move_back_callback)


    def move_back_callback(self, goal_handle):
        self.get_logger().info('def move_back_callback')

        try:
            movement_msg = Twist()

            speed = goal_handle.request.speed

            # ensure negative value to drive backwards
            if speed > 0:
                speed = -speed

            movement_msg.linear.x = speed 
            self._cmd_vel_publisher.publish(movement_msg)

            time.sleep(goal_handle.request.duration)

            stop_msg = Twist()
            stop_msg.linear.x = 0
            self._cmd_vel_publisher.publish(stop_msg)

            success = True
            goal_handle.succeed()

        except Exception as e:
            self.get_logger().error(f'move_back failed with error: {e}')
            success = False

        result = MoveBack.Result()
        result.success = success

        return result



def main(args=None):
    rclpy.init(args=args)

    product_aligner = MoveBackServer()
    
    rclpy.spin(product_aligner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()