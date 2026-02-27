from dataclasses import field, dataclass

from giskardpy.model.world_config import WorldWithOmniDriveRobot
from pkg_resources import resource_filename
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.robots.armar7 import Armar7

from giskardpy_ros.configs.robot_interface_config import RobotInterfaceConfig

class Armar7VelocityInterface(RobotInterfaceConfig):

    def setup(self):
        self.sync_joint_state_topic("/joint_states")
        joints = [
            "ArmR1_Cla1",
            "ArmR2_Sho1",
            "ArmR3_Sho2",
            "ArmR4_Sho3",
            "ArmR5_Elb1",
            "ArmR6_Elb2",
            "ArmR7_Wrist_Hemisphere_A",
            "ArmR8_Wrist_Hemisphere_B",
            "ArmL1_Cla1",
            "ArmL2_Sho1",
            "ArmL3_Sho2",
            "ArmL4_Sho3",
            "ArmL5_Elb1",
            "ArmL6_Elb2",
            "ArmL7_Wrist_Hemisphere_A",
            "ArmL8_Wrist_Hemisphere_B",
            "Ankle",
            "Knee",
            "Hip"
        ]
        self.add_joint_velocity_group_controller(
            cmd_topic="/realtime_body_controller_real/command", connections=joints
        )

@dataclass
class WorldWithArmar7Config(WorldWithOmniDriveRobot):
    urdf_view: AbstractRobot = field(kw_only=True, default=Armar7, init=False)

    def setup_collision_config(self):
        pass






