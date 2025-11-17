from __future__ import annotations

import traceback
from dataclasses import dataclass, field
from typing import List

import rclpy

from giskardpy.data_types.exceptions import SetupException
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.model.collision_world_syncer import (
    CollisionWorldSynchronizer,
    CollisionCheckerLib,
)
from giskardpy.model.collisions import NullCollisionDetector
from giskardpy.model.world_config import WorldConfig
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.motion_statechart.monitors.monitors import Monitor
from giskardpy.motion_statechart.tasks.task import Task
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.utils.utils import get_all_classes_in_package
from giskardpy_ros.configs.behavior_tree_config import BehaviorTreeConfig
from giskardpy_ros.configs.robot_interface_config import RobotInterfaceConfig
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from semantic_digital_twin.adapters.ros.world_fetcher import FetchWorldServer
from semantic_digital_twin.world_description.connections import ActiveConnection
from semantic_digital_twin.robots.abstract_robot import AbstractRobot

from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelReloadSynchronizer,
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.orm.ormatic_interface import *


@dataclass
class Giskard:
    """
    The main Class of Giskard.
    Instantiate it with appropriate configs for you setup and then call giskard.live()
    :param world_config: A world configuration. Use a predefined one or implement your own WorldConfig class.
    :param robot_interface_config: How Giskard talk to the robot. You probably have to implement your own RobotInterfaceConfig.
    :param collision_avoidance_config: default is no collision avoidance or implement your own collision_avoidance_config.
    :param behavior_tree_config: default is open loop mode
    :param qp_controller_config: default is good for almost all cases
    :param additional_goal_package_paths: specify paths that Giskard needs to import to find your custom Goals.
                                          Giskard will run 'from <additional path> import *' for each additional
                                          path in the list.
    :param additional_monitor_package_paths: specify paths that Giskard needs to import to find your custom Monitors.
                                          Giskard will run 'from <additional path> import *' for each additional
                                          path in the list.
    """

    world_config: WorldConfig
    behavior_tree_config: BehaviorTreeConfig
    robot_interface_config: RobotInterfaceConfig
    collision_checker_id: CollisionCheckerLib = CollisionCheckerLib.bpb
    qp_controller_config: QPControllerConfig = field(default_factory=QPControllerConfig)

    def __post_init__(self):
        god_map.tmp_folder = get_middleware().resolve_iri(
            "package://giskardpy_ros/tmp/"
        )
        GiskardBlackboard().giskard = self
        god_map.hack = 0

    def create_collision_detector(self, collision_checker: CollisionCheckerLib):
        if collision_checker not in CollisionCheckerLib:
            raise KeyError(
                f"Unknown collision checker {collision_checker}. "
                f"Collision avoidance is disabled"
            )
        if collision_checker == CollisionCheckerLib.bpb:
            get_middleware().loginfo("Using betterpybullet for collision checking.")
            try:
                from giskardpy.model.better_pybullet_syncer import (
                    BulletCollisionDetector,
                )

                return BulletCollisionDetector(_world=self.world_config.world)
            except ImportError as e:
                get_middleware().logerr(f"{e}; turning off collision avoidance.")
                self._collision_checker = CollisionCheckerLib.none
        get_middleware().logwarn("Using no collision checking.")
        return NullCollisionDetector(_world=self.world_config.world)

    def setup(self):
        """
        Initialize the behavior tree and world. You usually don't need to call this.
        """
        god_map.model_synchronizer = ModelSynchronizer(
            world=self.world_config.world, node=rospy.node
        )
        god_map.model_synchronizer.pause()
        god_map.state_synchronizer = StateSynchronizer(
            world=self.world_config.world, node=rospy.node
        )
        god_map.world_fetcher = FetchWorldServer(
            node=rospy.node, world=self.world_config.world
        )
        with self.world_config.world.modify_world():
            self.world_config.setup_world()
            god_map.world = self.world_config.world

            collision_detector = self.create_collision_detector(
                self.collision_checker_id
            )

            robots = self.world_config.world.get_semantic_annotations_by_type(
                AbstractRobot
            )
            self.collision_scene = CollisionWorldSynchronizer(
                collision_detector=collision_detector,
                world=self.world_config.world,
                robots=robots,
            )
            god_map.collision_scene = self.collision_scene
            self.qp_controller_config.setup()

            self.behavior_tree_config.setup()

            self.robot_interface_config.setup()
            self.world_config.setup_collision_config()

        if god_map.collision_scene.is_collision_checking_enabled():
            self.collision_scene.sync()

        self.sanity_check()
        GiskardBlackboard().tree.setup(rospy.node)

    def sanity_check(self):
        self._controlled_joints_sanity_check()

    @property
    def robot(self) -> AbstractRobot:
        return self.robots[0]

    @property
    def robots(self) -> List[AbstractRobot]:
        return self.world_config.world.get_semantic_annotations_by_type(AbstractRobot)

    def _controlled_joints_sanity_check(self):
        world = god_map.world
        movable_joints = world.get_connections_by_type(ActiveConnection)
        controlled_joints = self.robot.controlled_connections
        non_controlled_joints = set(movable_joints).difference(set(controlled_joints))
        # if len(controlled_joints) == 0 and len(world.connections) > 0:
        #     raise SetupException("No joints are flagged as controlled.")
        if len(non_controlled_joints) > 0:
            get_middleware().loginfo(
                f"The following joints are non-fixed according to the urdf, "
                f"but not flagged as controlled: {[c.name.name for c in non_controlled_joints]}."
            )

    def add_goal_package_name(self, package_name: str):
        new_goals = get_all_classes_in_package(package_name, Goal)
        if len(new_goals) == 0:
            raise SetupException(
                f"No classes of type '{Goal.__name__}' found in {package_name}."
            )
        get_middleware().loginfo(f"Made goal classes {new_goals} available.")
        god_map.motion_statechart_manager.add_goal_package_path(package_name)

    def add_task_package_name(self, package_name: str):
        new_goals = get_all_classes_in_package(package_name, Task)
        if len(new_goals) == 0:
            raise SetupException(
                f"No classes of type '{Goal.__name__}' found in {package_name}."
            )
        get_middleware().loginfo(f"Made task classes {new_goals} available.")
        god_map.motion_statechart_manager.add_task_package_path(package_name)

    def add_monitor_package_name(self, package_name: str) -> None:
        new_monitors = get_all_classes_in_package(package_name, Monitor)
        if len(new_monitors) == 0:
            raise SetupException(
                f"No classes of type '{Monitor.__name__}' found in '{package_name}'."
            )
        get_middleware().loginfo(f"Made Monitor classes '{new_monitors}' available.")
        god_map.motion_statechart_manager.add_monitor_package_path(package_name)

    def live(self):
        """
        Start Giskard.
        """
        try:
            self.setup()
            GiskardBlackboard().tree.live()
        except Exception as e:
            traceback.print_exc()
            rclpy.shutdown()
