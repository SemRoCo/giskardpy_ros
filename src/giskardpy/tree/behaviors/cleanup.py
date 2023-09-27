import rospy
from py_trees import Status
from visualization_msgs.msg import MarkerArray, Marker

from giskardpy import identifier
from giskardpy.debug_expression_manager import DebugExpressionManager
from giskardpy.goals.monitors.monitor_manager import MonitorManager
from giskardpy.goals.motion_goal_manager import MotionGoalManager
from giskardpy.god_map_user import GodMap
from giskardpy.model.collision_world_syncer import Collisions
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time


class CleanUp(GiskardBehavior):
    @profile
    def __init__(self, name, clear_markers=True):
        super().__init__(name)
        self.clear_markers_ = clear_markers
        self.marker_pub = rospy.Publisher('~visualization_marker_array', MarkerArray, queue_size=10)

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
        GodMap.god_map.clear_cache()
        GodMap.get_giskard().set_defaults()
        GodMap.get_world().fast_all_fks = None
        GodMap.get_collision_scene().reset_cache()
        GodMap.god_map.set_data(identifier.closest_point, Collisions(1))
        GodMap.god_map.set_data(identifier.time, 1)
        GodMap.god_map.set_data(identifier.monitor_manager, MonitorManager())
        GodMap.god_map.set_data(identifier.motion_goal_manager, MotionGoalManager())
        GodMap.god_map.set_data(identifier.debug_expression_manager, DebugExpressionManager())

        GodMap.god_map.set_data(identifier.next_move_goal, None)
        if hasattr(self.get_blackboard(), 'runtime'):
            del self.get_blackboard().runtime

    def update(self):
        return Status.SUCCESS


class CleanUpPlanning(CleanUp):
    def initialise(self):
        super().initialise()
        GodMap.god_map.set_data(identifier.fill_trajectory_velocity_values, None)


class CleanUpBaseController(CleanUp):
    pass
