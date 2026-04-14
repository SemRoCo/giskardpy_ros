from giskardpy_ros.configs.iai_robots.stretch import WorldWithStretchConfigDiffDrive, StretchVelocityInterface
from giskardpy.middleware.ros2 import rospy
from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from giskardpy_ros.utils.utils import load_xacro


def main():
    rospy.init_node("giskard")
    # try:
    #     rospy.node.declare_parameters(
    #         namespace="", parameters=[("robot_description", Parameter.Type.STRING)]
    #     )
    #     robot_description = rospy.node.get_parameter_or("robot_description").value
    # except ParameterUninitializedException as e:
    robot_description = load_xacro(
        "package://stretch_description/urdf/stretch_description_RE2V0_tool_stretch_dex_wrist.xacro"
    )
    giskard = Giskard(
        world_config=WorldWithStretchConfigDiffDrive(urdf=robot_description),
        robot_interface_config=StretchVelocityInterface(),
        behavior_tree_config=ClosedLoopBTConfig(
            visualization_mode=VisualizationMode.Nothing
        ),
        qp_controller_config=QPControllerConfig(
            target_frequency=25, prediction_horizon=30
        ),
    )
    giskard.live()


if __name__ == "__main__":
    main()