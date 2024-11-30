import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        my_parameter_descriptor = ParameterDescriptor(description='This is a tutorial parameter!')

        self.declare_parameter('my_parameter', 'default_parameter_value', my_parameter_descriptor)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Value of parameter: %s' % my_param)

        # # set parameter back to default value
        # my_new_param = rclpy.parameter.Parameter(
        #     'my_parameter',
        #     rclpy.Parameter.Type.STRING,
        #     'default_parameter_value'
        # )
        # all_new_parameters = [my_new_param]
        # self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()