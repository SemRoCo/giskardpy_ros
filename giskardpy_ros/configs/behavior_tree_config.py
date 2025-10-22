from dataclasses import dataclass, field
from typing import Optional

from py_trees.decorators import FailureIsSuccess

from giskardpy_ros.tree.behaviors.publish_debug_expressions import QPDataPublisherConfig
from giskardpy_ros.tree.branches.send_trajectories import ExecuteTraj

from giskardpy.data_types.exceptions import SetupException
from giskardpy.god_map import god_map
from giskardpy_ros.ros2.ros_msg_visualization import ROSMsgVisualization
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from giskardpy_ros.tree.behaviors.tf_publisher import TfPublishingModes
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
from giskardpy_ros.tree.branches.giskard_bt import GiskardBT
from giskardpy_ros.utils.utils import is_in_github_workflow


@dataclass
class BehaviorTreeConfig:
    tree: GiskardBT = field(init=False)
    tree_tick_rate: float = 0.05
    debug_mode: bool = False
    visualization_mode: VisualizationMode = VisualizationMode.CollisionsDecomposed

    add_gantt_chart_plotter: bool = False
    add_goal_graph_plotter: bool = False
    add_trajectory_plotter: bool = False
    add_debug_trajectory_plotter: bool = False
    add_debug_marker_publisher: bool = False
    add_trajectory_visualizer: bool = False
    add_debug_trajectory_visualizer: bool = False
    add_qp_data_publisher: QPDataPublisherConfig = field(
        default_factory=QPDataPublisherConfig
    )

    def __post_init__(self):
        if is_in_github_workflow():
            self.debug_mode = False
            self.visualization_mode = VisualizationMode.Nothing

    def is_closed_loop(self) -> bool:
        return isinstance(self, ClosedLoopBTConfig)

    def is_standalone(self) -> bool:
        return isinstance(self, StandAloneBTConfig)

    def is_open_loop(self) -> bool:
        return isinstance(self, OpenLoopBTConfig)

    def setup(self):
        """
        Implement this method to configure the behavior tree using it's self. methods.
        """
        GiskardBlackboard().tree_config = self
        self.tree = GiskardBT()
        if self.debug_mode:
            # self.add_gantt_chart_plotter()
            # self.add_goal_graph_plotter()
            if self.add_trajectory_plotter:
                self._add_trajectory_plotter(wait=True)
            if self.add_debug_trajectory_plotter:
                self._add_debug_trajectory_plotter(wait=True)
            if self.add_debug_marker_publisher:
                self._add_debug_marker_publisher()
            if self.add_trajectory_visualizer:
                self._add_trajectory_visualizer()
            if self.add_debug_trajectory_visualizer:
                self._add_debug_trajectory_visualizer()
            if self.add_gantt_chart_plotter:
                self._add_gantt_chart_plotter()
            if self.add_goal_graph_plotter:
                self._add_goal_graph_plotter()
            if self.add_qp_data_publisher.any():
                self._add_qp_data_publisher(publish_config=self.add_qp_data_publisher)

    def switch_to_projection_mode(self):
        """Override this method to define projection mode behavior for each config type."""
        raise NotImplementedError()

    def switch_to_execution_mode(self):
        """Override this method to define execution mode behavior for each config type."""
        raise NotImplementedError()

    def add_visualization_marker_publisher(
        self,
        mode: VisualizationMode,
        add_to_sync: Optional[bool] = None,
        add_to_control_loop: Optional[bool] = None,
        scale_scale: float = 1.0,
    ):
        """

        :param add_to_sync: Markers are published while waiting for a goal.
        :param add_to_control_loop: Markers are published during the closed loop control sequence, this is slow.
        :param use_decomposed_meshes: True: publish decomposed meshes used for collision avoidance, these likely only
                                            available on the machine where Giskard is running.
                                      False: use meshes defined in urdf.
        """
        GiskardBlackboard().ros_visualizer = ROSMsgVisualization(mode=mode)
        if add_to_sync:
            self.tree.wait_for_goal.publish_state.add_visualization_marker_behavior(
                mode, scale_scale=scale_scale
            )
        if add_to_control_loop:
            self.tree.control_loop_branch.publish_state.add_visualization_marker_behavior(
                mode, scale_scale=scale_scale
            )

    def _add_qp_data_publisher(self, publish_config: QPDataPublisherConfig):
        """
        QP data is streamed and can be visualized in e.g. plotjuggler. Useful for debugging.
        """
        self.add_evaluate_debug_expressions()
        if GiskardBlackboard().tree_config.is_open_loop():
            self.tree.execute_traj.base_closed_loop.publish_state.add_qp_data_publisher(
                publish_config=publish_config
            )
        else:
            self.tree.control_loop_branch.publish_state.add_qp_data_publisher(
                publish_config=publish_config
            )

    def _add_trajectory_plotter(
        self, normalize_position: bool = False, wait: bool = False
    ):
        """
        Plots the generated trajectories.
        :param normalize_position: Positions are centered around zero.
        :param wait: True: Behavior tree waits for this plotter to finish.
                     False: Plot is generated in a separate thread to not slow down Giskard.
        """
        self.tree.cleanup_control_loop.add_plot_trajectory(normalize_position, wait)

    def _add_trajectory_visualizer(self):
        self.tree.cleanup_control_loop.add_visualize_trajectory()

    def _add_debug_trajectory_visualizer(self):
        self.tree.cleanup_control_loop.add_debug_visualize_trajectory()

    def _add_debug_trajectory_plotter(
        self, normalize_position: bool = False, wait: bool = False
    ):
        """
        Plots debug expressions defined in goals.
        """
        self.add_evaluate_debug_expressions()
        self.tree.cleanup_control_loop.add_plot_debug_trajectory(
            normalize_position=normalize_position, wait=wait
        )

    def _add_gantt_chart_plotter(self):
        self.add_evaluate_debug_expressions()
        self.tree.cleanup_control_loop.add_plot_gantt_chart()

    def _add_goal_graph_plotter(self):
        self.add_evaluate_debug_expressions()
        self.tree.prepare_control_loop.add_plot_goal_graph()

    def _add_debug_marker_publisher(self):
        """
        Publishes debug expressions defined in goals.
        """
        self.add_evaluate_debug_expressions()
        self.tree.control_loop_branch.publish_state.add_debug_marker_publisher()

    def add_tf_publisher(
        self,
        include_prefix: bool = True,
        tf_topic: str = "tf",
        mode: TfPublishingModes = TfPublishingModes.attached_and_world_objects,
    ):
        """
        Publishes tf for Giskard's internal state.
        """
        self.tree.wait_for_goal.publish_state.add_tf_publisher(
            include_prefix=include_prefix, tf_topic=tf_topic, mode=mode
        )
        if GiskardBlackboard().tree_config.is_standalone():
            self.tree.control_loop_branch.publish_state.add_tf_publisher(
                include_prefix=include_prefix, tf_topic=tf_topic, mode=mode
            )

    def add_robot_description_publisher(self, topic: str = "robot_description"):
        if GiskardBlackboard().tree_config.is_standalone():
            self.tree.wait_for_goal.publish_state.add_robot_description_publisher(
                topic=topic
            )

    def add_evaluate_debug_expressions(self):
        self.tree.prepare_control_loop.add_compile_debug_expressions()
        if GiskardBlackboard().tree_config.is_closed_loop():
            self.tree.control_loop_branch.add_evaluate_debug_expressions(log_traj=False)
        else:
            self.tree.control_loop_branch.add_evaluate_debug_expressions(log_traj=True)
        if GiskardBlackboard().tree_config.is_open_loop() and hasattr(
            GiskardBlackboard().tree.execute_traj, "prepare_base_control"
        ):
            GiskardBlackboard().tree.execute_traj.prepare_base_control.add_compile_debug_expressions()
            GiskardBlackboard().tree.execute_traj.base_closed_loop.add_evaluate_debug_expressions(
                log_traj=False
            )

    def add_js_publisher(
        self, topic_name: Optional[str] = None, include_prefix: bool = False
    ):
        """
        Publishes joint states for Giskard's internal state.
        """
        GiskardBlackboard().tree.control_loop_branch.publish_state.add_joint_state_publisher(
            include_prefix=include_prefix,
            topic_name=topic_name,
            only_prismatic_and_revolute=True,
        )
        GiskardBlackboard().tree.wait_for_goal.publish_state.add_joint_state_publisher(
            include_prefix=include_prefix,
            topic_name=topic_name,
            only_prismatic_and_revolute=True,
        )

    def add_free_variable_publisher(
        self, topic_name: Optional[str] = None, include_prefix: bool = False
    ):
        """
        Publishes joint states for Giskard's internal state.
        """
        GiskardBlackboard().tree.control_loop_branch.publish_state.add_joint_state_publisher(
            include_prefix=include_prefix,
            topic_name=topic_name,
            only_prismatic_and_revolute=False,
        )
        GiskardBlackboard().tree.wait_for_goal.publish_state.add_joint_state_publisher(
            include_prefix=include_prefix,
            topic_name=topic_name,
            only_prismatic_and_revolute=False,
        )


@dataclass
class StandAloneBTConfig(BehaviorTreeConfig):
    """
    The default behavior tree for Giskard in standalone mode. Make sure to set up the robot interface accordingly.
    :param publish_js: publish current world state.
    :param publish_tf: publish all link poses in tf.
    :param include_prefix: whether to include the robot name prefix when publishing joint states or tf
    """

    publish_js: bool = False
    publish_free_variables: bool = False
    publish_tf: bool = True
    include_prefix: bool = False
    publish_robot_description: bool = True
    visualization_mode: VisualizationMode = VisualizationMode.VisualsFrameLocked

    def __post_init__(self):
        super().__post_init__()
        if is_in_github_workflow():
            self.publish_js = False
            self.publish_tf = True
        if self.publish_js and self.publish_free_variables:
            raise SetupException(
                "publish_js and publish_free_variables cannot be True at the same time."
            )

    def setup(self):
        super().setup()
        self.tree.control_loop_branch.add_projection_behaviors()
        self.add_visualization_marker_publisher(
            add_to_sync=True, add_to_control_loop=True, mode=self.visualization_mode
        )
        if self.publish_tf:
            self.add_tf_publisher(
                include_prefix=self.include_prefix, mode=TfPublishingModes.all
            )
        if self.publish_robot_description:
            self.add_robot_description_publisher()
        self.add_evaluate_debug_expressions()
        self._add_debug_trajectory_plotter()
        if self.publish_js:
            self.add_js_publisher(include_prefix=self.include_prefix)
        if self.publish_free_variables:
            self.add_free_variable_publisher(include_prefix=False)

    def switch_to_projection_mode(self):
        # StandAlone specific projection logic
        pass

    def switch_to_execution_mode(self):
        # StandAlone specific execution logic
        pass


@dataclass
class OpenLoopBTConfig(BehaviorTreeConfig):
    """
    The default behavior tree for Giskard in open-loop mode. It will first plan the trajectory in simulation mode
    and then publish it to connected joint trajectory followers. The base trajectory is tracked with a closed-loop
    controller.
    """

    def setup(self):
        super().setup()
        self.tree.control_loop_branch.add_projection_behaviors()
        self.tree.execute_traj = ExecuteTraj()
        self.tree.execute_traj_failure_is_success = FailureIsSuccess(
            "ignore failure", self.tree.execute_traj
        )
        self.add_visualization_marker_publisher(
            add_to_sync=True, add_to_control_loop=True, mode=self.visualization_mode
        )

    def switch_to_projection_mode(self):
        self.tree.root.remove_child(self.tree.execute_traj_failure_is_success)

    def switch_to_execution_mode(self):
        self.tree.root.insert_child(self.tree.execute_traj_failure_is_success, -2)


@dataclass
class ClosedLoopBTConfig(BehaviorTreeConfig):
    """
    The default configuration for Giskard in closed loop mode. Make use to set up the robot interface accordingly.
    """

    def setup(self):
        super().setup()
        self.tree.control_loop_branch.add_closed_loop_behaviors()
        self.add_visualization_marker_publisher(
            add_to_sync=True, add_to_control_loop=False, mode=self.visualization_mode
        )

    def switch_to_projection_mode(self):
        self.tree.control_loop_branch.switch_to_projection()

    def switch_to_execution_mode(self):
        self.tree.control_loop_branch.switch_to_closed_loop()
