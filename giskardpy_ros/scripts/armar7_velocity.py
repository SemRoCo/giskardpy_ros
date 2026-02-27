from giskardpy.model.collision_world_syncer import CollisionCheckerLib

from giskardpy_ros.configs.iai_robots.tiago import TiagoVelocityInterface, WorldWithTiagoConfig
from giskardpy_ros.configs.other_robots.armar7 import WorldWithArmar7Config, Armar7VelocityInterface
from giskardpy_ros.ros2 import rospy
from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig, StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from giskardpy_ros.utils.utils import load_xacro
from giskardpy_ros.configs.robot_interface_config import StandAloneRobotInterfaceConfig


def main():
    rospy.init_node("giskard")

    rospy.node.declare_parameters(
        namespace="", parameters=[("robot_description", Parameter.Type.STRING)]
    )
    robot_description = rospy.node.get_parameter_or("robot_description").value

    giskard = Giskard(
        world_config=WorldWithArmar7Config(urdf=robot_description),
        collision_checker_id=CollisionCheckerLib.none,
        robot_interface_config=Armar7VelocityInterface(),
        behavior_tree_config=ClosedLoopBTConfig(
            visualization_mode=VisualizationMode.VisualsFrameLocked
        ),
        qp_controller_config=QPControllerConfig(
            target_frequency=80, prediction_horizon=7
        ),
    )
    giskard.live()


if __name__ == "__main__":
    main()