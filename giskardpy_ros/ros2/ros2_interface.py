import os
import asyncio
from typing import List, Type, Optional, Tuple, Union, Any

import rclpy
import xacro
from ament_index_python import get_package_share_directory
from controller_manager import controller_manager_services
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rcl_interfaces.srv import GetParameters_Request, GetParameters_Response, GetParameters
from rclpy import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.wait_for_message import wait_for_message as rclpy_wait_for_message
from std_msgs.msg import String

from giskardpy.middleware import get_middleware
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.ros2.msg_converter import msg_type_as_str
from giskardpy_ros.utils.asynio_utils import wait_until_not_none


def wait_for_topic_to_appear(topic_name: str,
                             supported_types=None,
                             sleep_time: float = 1):
    rclpy.wait_for_message.wait_for_message()
    waiting_message = f'Waiting for topic \'{topic_name}\' to appear...'
    msg_type = None
    while msg_type is None and not rospy.is_shutdown():
        get_middleware().loginfo(waiting_message)
        try:
            rostopic.get_info_text(topic_name)
            msg_type, _, _ = rostopic.get_topic_class(topic_name)
            if msg_type is None:
                raise ROSTopicException()
            if supported_types is not None and msg_type not in supported_types:
                raise TypeError(f'Topic of type \'{msg_type}\' is not supported. '
                                f'Must be one of: \'{supported_types}\'')
            else:
                get_middleware().loginfo(f'\'{topic_name}\' appeared.')
                return msg_type
        except (ROSException, ROSTopicException) as e:
            rospy.sleep(sleep_time)


def wait_for_message(msg_type,
                     node: 'Node',
                     topic: str,
                     *,
                     qos_profile: Union[QoSProfile, int] = 10,
                     time_to_wait=-1) -> Tuple[bool, Any]:
    while True:
        try:
            result = rclpy_wait_for_message(msg_type=msg_type, node=node, topic=topic, qos_profile=qos_profile)
            if result[1] is not None:
                return result
        except Exception as e:
            node.get_logger().info(f'waiting for message from {topic}.')


def get_robot_description(topic: str = '/robot_description') -> str:
    qos_profile = QoSProfile(depth=10)
    qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return wait_for_message(String, rospy.node, topic, qos_profile=qos_profile)[1].data


def search_for_publisher_of_node_with_type(node_name: str, topic_type):
    topics = rospy.node.get_publisher_names_and_types_by_node(node_name, '/')
    return _search_in_topic_list(node_name=node_name, topic_list=topics, topic_type=topic_type)[0]


def search_for_subscriber_of_node_with_type(node_name: str, topic_type):
    topics = rospy.node.get_subscriber_names_and_types_by_node(node_name, '/')
    return _search_in_topic_list(node_name=node_name, topic_list=topics, topic_type=topic_type)[0]


def search_for_publishers_of_type(topic_type) -> List[str]:
    topics = _search_in_topic_list(topic_list=rospy.node.get_topic_names_and_types(), topic_type=topic_type)
    matches = []
    for topic_name in topics:
        if len(rospy.node.get_publishers_info_by_topic(topic_name)) > 0:
            matches.append(topic_name)
    return matches


def search_for_unique_publisher_of_type(topic_type) -> str:
    topic_names = search_for_publishers_of_type(topic_type)
    assert len(topic_names) == 1, f'Found too many {msg_type_as_str(topic_type)} topics: {topic_names}.'
    return topic_names[0]


def search_for_unique_subscriber_of_type(topic_type) -> str:
    topic_names = search_for_subscribers_of_type(topic_type)
    assert len(topic_names) == 1, f'Found too many {msg_type_as_str(topic_type)} topics: {topic_names}.'
    return topic_names[0]


def search_for_subscribers_of_type(topic_type) -> List[str]:
    topics = _search_in_topic_list(topic_list=rospy.node.get_topic_names_and_types(), topic_type=topic_type)
    matches = []
    for topic_name in topics:
        if len(rospy.node.get_subscriptions_info_by_topic(topic_name)) > 0:
            matches.append(topic_name)
    return matches


def get_parameters(parameters: List[str], node_name: str = 'controller_manager') -> GetParameters_Response:
    req = GetParameters_Request()
    req.names = parameters
    return controller_manager_services.service_caller(node=rospy.node,
                                                      service_name=f'{node_name}/get_parameters',
                                                      service_type=GetParameters,
                                                      request=req,
                                                      service_timeout=10)


def _search_in_topic_list(topic_list: List[Tuple[str, list]], topic_type: str, node_name: Optional[str] = None) \
        -> List[str]:
    matches = []
    for topic_name, topic_types in topic_list:
        if topic_types[0] == msg_type_as_str(topic_type):
            matches.append(topic_name)
    if matches:
        return matches
    if node_name is not None:
        raise AttributeError(f'Node {node_name} has no topic of type {topic_type}.')
    else:
        raise AttributeError(f'Didn\'t find topic of type {topic_type}.')


def wait_for_publisher(publisher):
    return
    # while publisher.get_num_connections() == 0:
    #     rospy.sleep(0.1)


def load_urdf(file_path: str) -> str:
    file_path = get_middleware().resolve_iri(file_path)
    doc = xacro.process_file(file_path, mappings={'radius': '0.9'})
    return doc.toprettyxml(indent='  ')


class ControllerManager:
    name: str

    def __init__(self, name: str = 'controller_manager'):
        self.name = name


class MyActionClient:
    _goal_handle: Optional[ClientGoalHandle]
    _result_future: Optional[Future]

    def __init__(self, node_handle: Node, action_type, action_name: str):
        self._goal_handle = None
        self._goal_result = None
        self._result_future = None
        self.node_handle = node_handle
        self._client = ActionClient(node=node_handle,
                                    action_type=action_type,
                                    action_name=action_name)
        self._client.wait_for_server()

    @property
    def _event_loop(self) -> asyncio.AbstractEventLoop:
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.new_event_loop()

    def send_goal_async(self, goal) -> Future:
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.__goal_accepted_cb)
        return future

    def send_goal(self, goal):
        # this works in theory too, but not in pycharms debugger, that's why i'm putting it into a function
        # self._event_loop.run_until_complete(self.send_goal_async(goal))
        # return self._event_loop.run_until_complete(self.get_result())
        async def muh():
            await self.send_goal_async(goal)
            return await self.get_result()

        return self._event_loop.run_until_complete(muh())

    async def get_result(self):
        await wait_until_not_none(lambda: self._result_future)
        result = await self._result_future
        self._result_future = None
        return result.result

    def __goal_accepted_cb(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node_handle.get_logger().info('Goal rejected')
            return

        self._goal_handle = goal_handle
        self.node_handle.get_logger().info('Goal accepted')

        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self.__goal_done_cb)

    def __goal_done_cb(self, future: Future):
        self.node_handle.get_logger().info(f'Goal result received')
        self._goal_handle = None
