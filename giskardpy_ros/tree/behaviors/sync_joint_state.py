from typing import Optional

from py_trees.common import Status
from sensor_msgs.msg import JointState

import giskardpy_ros.ros2.msg_converter as msg_converter
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.utils.decorators import record_time
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from semantic_digital_twin.spatial_types.derivatives import Derivatives
from semantic_digital_twin.world_description.world_state import WorldState


class SyncJointState(GiskardBehavior):

    @record_time
    def __init__(self, group_name: str, joint_state_topic: str = "joint_states"):
        self.data = None
        self.group_name = group_name
        self.joint_state_topic = joint_state_topic
        if not self.joint_state_topic.startswith("/"):
            self.joint_state_topic = "/" + self.joint_state_topic
        super().__init__(str(self))

    @record_time
    def setup(self, **kwargs):
        # wait_for_topic_to_appear(topic_name=self.joint_state_topic, supported_types=[JointState])
        self.joint_state_sub = rospy.node.create_subscription(
            JointState, self.joint_state_topic, self.cb, 1
        )
        return super().setup(**kwargs)

    def cb(self, data):
        self.data = data

    @record_time
    def update(self):
        if self.data:
            mjs = msg_converter.ros_joint_state_to_giskard_joint_state(
                self.data, self.group_name
            )
            god_map.world.state.update(mjs)
            self.data = None
            return Status.SUCCESS
        return Status.RUNNING

    def __str__(self):
        return f"{super().__str__()} ({self.joint_state_topic})"


class SyncJointStatePosition(GiskardBehavior):
    """
    Listens to a joint state topic, transforms it into a dict and writes it to the got map.
    Gets replace with a kinematic sim plugin during a parallel universe.
    """

    msg: JointState

    @record_time
    def __init__(self, group_name: str, joint_state_topic="joint_states"):
        super().__init__(str(self))
        self.joint_state_topic = joint_state_topic
        if not self.joint_state_topic.startswith("/"):
            self.joint_state_topic = "/" + self.joint_state_topic
        # wait_for_topic_to_appear(topic_name=self.joint_state_topic, supported_types=[JointState])
        super().__init__(str(self))
        self.mjs: Optional[WorldState] = None
        self.group_name = group_name

    @record_time
    def setup(self, **kwargs):
        self.joint_state_sub = rospy.node.create_subscription(
            JointState, self.joint_state_topic, self.cb, 1
        )
        get_middleware().loginfo(f"Subscribed to {self.joint_state_topic}")
        return super().setup(**kwargs)

    def cb(self, data):
        self.msg = data

    def initialise(self):
        self.last_time = rospy.node.get_clock().now()
        super().initialise()

    @record_time
    def update(self):
        for joint_name, position in zip(self.msg.name, self.msg.position):
            joint_name = PrefixedName(joint_name, self.group_name)
            god_map.world.state[joint_name][Derivatives.position] = position

        return Status.SUCCESS
