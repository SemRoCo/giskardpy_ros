from py_trees.common import Status
from visualization_msgs.msg import MarkerArray, Marker

from giskardpy.god_map import god_map
from giskardpy.model.collision_world_syncer import Collisions
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from giskardpy_ros.tree.blackboard_utils import catch_and_raise_to_blackboard, GiskardBlackboard
from line_profiler import profile


class CleanUp(GiskardBehavior):
    @profile
    def __init__(self, name, clear_markers=False):
        super().__init__(name)
        self.clear_markers_ = clear_markers
        self.marker_pub = rospy.node.create_publisher(MarkerArray,
                                                    f'{rospy.node.get_name()}/visualization_marker_array',
                                                10)

    def clear_markers(self):
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        self.marker_pub.publish(msg)

    @record_time
    @profile
    def initialise(self):
        if self.clear_markers_:
            self.clear_markers()
        GiskardBlackboard().giskard.set_defaults()
        if GiskardBlackboard().tree.control_loop_branch.publish_state.debug_marker_publisher is not None:
            self.clear_markers()
            GiskardBlackboard().ros_visualizer.publish_markers(force=True)
        god_map.world.compiled_all_fks = None
        god_map.collision_scene.reset_cache()
        god_map.collision_scene.clear_collision_matrix()
        god_map.closest_point = Collisions(1)
        god_map.time = 0
        god_map.control_cycle_counter = 1
        god_map.monitor_manager.reset()
        god_map.motion_goal_manager.reset()
        god_map.debug_expression_manager.reset()

        self.get_blackboard().runtime = None

    def update(self):
        return Status.SUCCESS


class CleanUpPlanning(CleanUp):
    def initialise(self):
        super().initialise()
        GiskardBlackboard().fill_trajectory_velocity_values = None
        god_map.free_variables = []

    @catch_and_raise_to_blackboard
    def update(self):
        return super().update()
