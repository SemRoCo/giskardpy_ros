from enum import Enum

from py_trees.common import Status
from tf2_msgs.msg import TFMessage

from giskardpy.middleware.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy_ros.tree.blackboard_utils import (
    GiskardBlackboard,
    catch_and_raise_to_blackboard,
)


class TFPublisher(GiskardBehavior):
    """
    Published tf for attached and environment objects.
    """

    @catch_and_raise_to_blackboard
    def update(self):
        GiskardBlackboard().giskard.tf_publisher._notify()
        return Status.SUCCESS
