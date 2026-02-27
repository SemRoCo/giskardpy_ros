from dataclasses import dataclass, field

from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy_ros.configs.giskard import RobotInterfaceConfig
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
)


@dataclass
class GenericWorldConfig(WorldWithFixedRobot):
    robot_name: str = field(kw_only=True, default="generic_robot")
    controller_manager_name: str = field(kw_only=True, default="controller_manager")


class GenericRobotInterface(RobotInterfaceConfig):
    drive_joint_name: str

    def __init__(self, controller_manager_name: str = "controller_manager"):
        self.controller_manager_name = controller_manager_name

    def setup(self):
        if GiskardBlackboard().tree_config.is_standalone():
            self.register_controlled_joints(
                [
                    c.name
                    for c in self.world.get_connections_by_type(ActiveConnection1DOF)
                ]
            )
        elif GiskardBlackboard().tree_config.is_closed_loop():
            self.discover_interfaces_from_controller_manager()
            # try:
            #     self.world.get_connections_by_type(OmniDrive)
            #     self.sync_odometry_topic()
            #     self.add_base_cmd_velocity()
            # except Exception as e:
            #     # no drive joint, so no need to add odom and cmd vel topic
            #     pass
        else:
            raise NotImplementedError("this mode is not implemented yet")
