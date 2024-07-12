from typing import List, Type, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import String

from giskardpy.middleware import middleware
from giskardpy_ros.ros2 import rospy


def wait_for_topic_to_appear(topic_name: str,
                             supported_types = None,
                             sleep_time: float = 1) :
    waiting_message = f'Waiting for topic \'{topic_name}\' to appear...'
    msg_type = None
    while msg_type is None and not rospy.is_shutdown():
        middleware.loginfo(waiting_message)
        try:
            rostopic.get_info_text(topic_name)
            msg_type, _, _ = rostopic.get_topic_class(topic_name)
            if msg_type is None:
                raise ROSTopicException()
            if supported_types is not None and msg_type not in supported_types:
                raise TypeError(f'Topic of type \'{msg_type}\' is not supported. '
                                f'Must be one of: \'{supported_types}\'')
            else:
                middleware.loginfo(f'\'{topic_name}\' appeared.')
                return msg_type
        except (ROSException, ROSTopicException) as e:
            rospy.sleep(sleep_time)


def get_robot_description(topic: str = '/robot_description') -> str:
    qos_profile = QoSProfile(depth=10)
    qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return wait_for_message(String, rospy.node, topic, qos_profile=qos_profile)[1].data


def wait_for_publisher(publisher):
    return
    # while publisher.get_num_connections() == 0:
    #     rospy.sleep(0.1)
