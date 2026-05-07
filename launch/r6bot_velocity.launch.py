from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # tracy_xacro_file = os.path.join(get_package_share_directory('iai_tracy_description'), 'urdf',
    #                                 'tracy.urdf.xacro')
    # robot_description = Command(
    #     [FindExecutable(name='xacro'), ' ', tracy_xacro_file])

    return LaunchDescription(
        [
            # Static transform publisher (example, modify as needed for your robot)
            # IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource(upload_pr2_launch)
            # # ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            # ),
            Node(
                package="giskardpy_ros",
                executable="r6bot",
                name="giskard",
                # parameters=[{'robot_description': robot_description}],
                output="screen",
            ),
            Node(
                package="giskardpy_ros",
                executable="interactive_marker",
                name="giskard_interactive_marker",
                parameters=[
                    {
                        "root_links": ["base_link", "map"],
                        "tip_links": ["tool0", "base_link"],
                    },
                ],
                output="screen",
            ),
            # RViz node
        ]
    )
