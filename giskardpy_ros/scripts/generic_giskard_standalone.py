from giskardpy.middleware.ros2 import rospy
from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.generic import (
    GenericWorldConfig,
    GenericRobotInterface,
)


def main():
    rospy.init_node("giskard")
    try:
        rospy.node.declare_parameters(
            namespace="", parameters=[("robot_description", Parameter.Type.STRING)]
        )
        robot_description = rospy.node.get_parameter_or("robot_description").value
    except ParameterUninitializedException as e:
        robot_description = None
    giskard = Giskard(
        world_config=GenericWorldConfig(robot_description=robot_description),
        robot_interface_config=GenericRobotInterface(),
        behavior_tree_config=StandAloneBTConfig(
            publish_free_variables=False,
            publish_tf=True,
            debug_mode=True,
            publish_world_state=True,
        ),
        qp_controller_config=QPControllerConfig(mpc_dt=0.05),
    )
    giskard.live()


if __name__ == "__main__":
    main()
