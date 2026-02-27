from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.hsr import (
    WorldWithHSRConfig,
    HSRVelocityInterface,
)
from giskardpy.middleware.ros2 import rospy
from giskardpy_ros.utils.utils import load_xacro


def main():
    rospy.init_node("giskard")
    urdf = load_xacro("package://hsr_description/robots/hsrb4s.urdf.xacro")
    # urdf = get_robot_description()
    giskard = Giskard(
        world_config=WorldWithHSRConfig(urdf=urdf),
        robot_interface_config=HSRVelocityInterface(),
        qp_controller_config=QPControllerConfig(
            target_frequency=40, prediction_horizon=15
        ),
        behavior_tree_config=ClosedLoopBTConfig(debug_mode=False),
    )
    giskard.live()


if __name__ == "__main__":
    main()
