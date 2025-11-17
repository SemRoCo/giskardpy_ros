from typing import Optional

import numpy as np

from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy_ros.configs.robot_interface_config import (
    RobotInterfaceConfig,
    StandAloneRobotInterfaceConfig,
)
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from semantic_digital_twin.spatial_types.derivatives import Derivatives
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.adapters.urdf import URDFParser


class TracyVelocityInterface(RobotInterfaceConfig):
    def __init__(self, controller_manager_name: str = 'controller_manager'):
        self.controller_manager_name = controller_manager_name

    def setup(self):
        self.sync_joint_state_topic('/left_arm/joint_states')
        self.sync_joint_state_topic('/right_arm/joint_states')
        self.sync_joint_state_topic('/right_gripper/joint_states')
        self.sync_joint_state_topic('/left_gripper/joint_states')
        joints_left = ['left_shoulder_pan_joint',
                       'left_shoulder_lift_joint',
                       'left_elbow_joint',
                       'left_wrist_1_joint',
                       'left_wrist_2_joint',
                       'left_wrist_3_joint']
        self.add_joint_velocity_group_controller(cmd_topic='/left_arm/forward_velocity_controller/commands', joints=joints_left)
        joints_right = ['right_shoulder_pan_joint',
                       'right_shoulder_lift_joint',
                       'right_elbow_joint',
                       'right_wrist_1_joint',
                       'right_wrist_2_joint',
                       'right_wrist_3_joint']
        self.add_joint_velocity_group_controller(cmd_topic='/right_arm/forward_velocity_controller/commands', joints=joints_right)

class WorldWithTracyConfig(WorldWithFixedRobot):
    """Minimal Tracy world config analogous to WorldWithPR2Config.

    - Fixed-base robot (no drive joint)
    - Accepts URDF via argument; if not provided, reads from ROS parameter server
    - Applies conservative default motion limits
    """

    def setup_collision_config(self):
        pass

    def __init__(self, urdf: Optional[str] = None, map_name: str = "map"):
        super().__init__(
            urdf=urdf, map_name=map_name
        )

    def setup_world(self, robot_name: Optional[str] = None) -> None:
        urdf_parser = URDFParser(urdf=self.urdf)
        world_with_robot = urdf_parser.parse()
        self.world = world_with_robot
        self.tracy = Tracy.from_world(world=self.world)


# class TracyCollisionAvoidanceConfig(LoadSelfCollisionMatrixConfig):
#     def __init__(self, collision_checker: CollisionCheckerLib = CollisionCheckerLib.bpb):
#         super().__init__('package://giskardpy_ros/self_collision_matrices/iai/tracy.srdf',
#                          collision_checker)


class TracyJointTrajServerMujocoInterface(RobotInterfaceConfig):
    def setup(self):
        self.sync_joint_state_topic('joint_states')
        self.add_follow_joint_trajectory_server(
            namespace='/left_arm/scaled_pos_joint_traj_controller_left')
        self.add_follow_joint_trajectory_server(
            namespace='/right_arm/scaled_pos_joint_traj_controller_right')


class TracyStandAloneRobotInterfaceConfig(StandAloneRobotInterfaceConfig):
    def __init__(self):
        super().__init__([
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
            'right_shoulder_pan_joint',
            'right_shoulder_lift_joint',
            'right_elbow_joint',
            'right_wrist_1_joint',
            'right_wrist_2_joint',
            'right_wrist_3_joint',
        ])
