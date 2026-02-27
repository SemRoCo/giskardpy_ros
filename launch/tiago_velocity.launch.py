import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    tracy_xacro_file = os.path.join(get_package_share_directory('iai_tiago_description'), 'urdf',
                                    'tiago_dual_pal_gripper.urdf')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', "/home/repo/tiago_dual_absolute_meshes.urdf"])

    return LaunchDescription([
        # Static transform publisher (example, modify as needed for your robot)
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(upload_pr2_launch)
        #),

        Node(
            package='giskardpy_ros',
            executable='tiago_velocity',
            name='giskard',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='giskardpy_ros',
            executable='interactive_marker',
            name='giskard_interactive_marker',
            parameters=[{'root_link': 'map',
                         'tip_link': 'arm_right_7_link'}],
            output='screen',
        ),
        # RViz node

    ])

