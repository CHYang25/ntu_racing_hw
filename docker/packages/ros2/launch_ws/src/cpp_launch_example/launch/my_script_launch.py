import launch
import launch_ros.actions

def generate_launch_description(): # this function must be named like this, and the return type is fixed as well
    return launch.LaunchDescription([
        launch_ros.actions.Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker'),
    ])