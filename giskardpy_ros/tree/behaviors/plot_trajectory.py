import traceback
from threading import Thread

from line_profiler import profile
from py_trees.common import Status
from rclpy.executors import MultiThreadedExecutor

from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from line_profiler import profile


class PlotTrajectory(GiskardBehavior):
    plot_thread = None
    plot_done = False


    def __init__(self, name, wait=False, joint_filter=None, normalize_position: bool = False, **kwargs):
        super().__init__(name)
        self.wait = wait
        self.normalize_position = normalize_position
        self.kwargs = kwargs
        self.path_to_data_folder = god_map.tmp_folder


    def initialise(self):
        # self.plot_thread = Thread(target=self.plot, name=self.name)
        # self.plot_thread.start()
        self.plot_thread = rospy.node.create_timer(0.0, self.plot)

    def plot(self):
        trajectory = god_map.trajectory
        if trajectory:
            sample_period = god_map.qp_controller.config.mpc_dt
            try:
                trajectory.plot_trajectory(path_to_data_folder=self.path_to_data_folder,
                                           sample_period=sample_period,
                                           normalize_position=self.normalize_position,
                                           **self.kwargs)
            except Exception as e:
                traceback.print_exc()
                get_middleware().logwarn(e)
                get_middleware().logwarn('failed to save trajectory.pdf')

    @record_time
    
    def update(self):
        if self.wait and not self.plot_done:
            return Status.RUNNING
        return Status.SUCCESS
