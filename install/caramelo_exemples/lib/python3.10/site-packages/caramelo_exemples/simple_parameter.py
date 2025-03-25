import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameters(Node):
    def __init__(self):
        super().__init__('simple_parameters')

        # Declare parameters é utilizado para declarar os parâmetros que serão utilizados pelo Nó
        self.declare_parameter("simple_int_param", 20)
        self.declare_parameter("simple_string_param", "Hello World")

        # add_on_set_parameters_callback é utilizado para adicionar um callback que será chamado quando um parâmetro for alterado
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    # quando um parâmetro for alterado, este método será chamado e alterará o valor do parâmetro
    def paramChangeCallback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Changing simple_int_param from %d to %d" % param.value)
                result.successful = True

            elif param.name == "simple_string_param":
                self.get_logger().info("Changing simple_string_param from %s to %s" % param.value)
                result.successful = True
            
            else:
                result.successful = False

        return result
    

def main():
    rclpy.init()
    simple_parameter = SimpleParameters()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()