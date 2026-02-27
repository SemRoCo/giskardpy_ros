#!/usr/bin/env python3

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import (
    ClosedLoopBTConfig,
)
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.generic import (
    GenericWorldConfig,
    GenericRobotInterface,
)
from giskardpy.middleware.ros2 import rospy
from giskardpy_ros.ros2.ros2_interface import get_robot_description


def main():
    rospy.init_node("giskard")
    robot_description = get_robot_description()
    giskard = Giskard(
        world_config=GenericWorldConfig(urdf=robot_description),
        robot_interface_config=GenericRobotInterface(),
        behavior_tree_config=ClosedLoopBTConfig(),
        qp_controller_config=QPControllerConfig(control_dt=0.0125, mpc_dt=0.0125),
    )
    giskard.live()


if __name__ == "__main__":
    main()
