import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    # Define package and URDF file paths
    package_description = "jumping_description"
    urdf_file = "jumping/robot.urdf"

    print('fetching urdf...')
    robot_desc_path = os.path.join(get_package_share_directory(package_description), urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"The specified URDF file does not exist: {robot_desc_path}")


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        #parameters=[{'use_sim_time':True, 'robot_description':Command(['xacro', robot_desc_path])}],
        parameters=[{'use_sim_time': True, 'robot_description': open(robot_desc_path).read()}],
        output="screen"
    )

    return LaunchDescription(
        [robot_state_publisher_node]
    )

if __name__ == '__main__':
    generate_launch_description()