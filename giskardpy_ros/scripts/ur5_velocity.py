
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import (
    ClosedLoopBTConfig,
    StandAloneBTConfig,
)
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.ur import WorldWithUR5Config, UR5VelocityInterface
from giskardpy.middleware.ros2 import rospy
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from giskardpy_ros.utils.utils import load_xacro
from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException

def main():
    rospy.init_node("giskard")

    rospy.node.declare_parameters(
        namespace="", parameters=[("robot_description", Parameter.Type.STRING)]
    )
    robot_description = rospy.node.get_parameter_or("robot_description").value

    giskard = Giskard(
        world_config=WorldWithUR5Config(urdf=robot_description),
        robot_interface_config=UR5VelocityInterface(),
        behavior_tree_config=ClosedLoopBTConfig(visualization_mode=VisualizationMode.VisualsFrameLocked),
        qp_controller_config=QPControllerConfig(
            target_frequency=80, prediction_horizon=7
        ),
    )
    giskard.live()

if __name__ == "__main__":
    main()