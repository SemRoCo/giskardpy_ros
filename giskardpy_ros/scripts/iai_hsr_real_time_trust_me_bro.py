from giskardpy.model.world_config import WorldFromDatabaseConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import ClosedLoopBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.hsr import (
    WorldWithHSRConfig,
    HSRVelocityInterface,
)
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.ros2.ros2_interface import get_robot_description
from giskardpy_ros.utils.utils import load_xacro


def main():
    rospy.init_node("giskard")
    giskard = Giskard(
        world_config=WorldFromDatabaseConfig(primary_key=1),
        robot_interface_config=HSRVelocityInterface(),
        qp_controller_config=QPControllerConfig(
            target_frequency=40, prediction_horizon=15
        ),
        behavior_tree_config=ClosedLoopBTConfig(debug_mode=False),
    )
    giskard.live()


if __name__ == "__main__":
    main()
