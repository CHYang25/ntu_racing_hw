from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",

            # we ensure our output is printed in our console.
            output="screen",
            emulate_tty=True,

            
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])