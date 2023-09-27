from enum import Enum

import rospy
from geometry_msgs.msg import TransformStamped
from py_trees import Status
from tf2_msgs.msg import TFMessage

from giskardpy.god_map_user import GodMap
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.utils.decorators import record_time
from giskardpy.utils.tfwrapper import normalize_quaternion_msg


class TfPublishingModes(Enum):
    nothing = 0
    all = 1
    attached_objects = 2

    world_objects = 4
    attached_and_world_objects = 6

class TFPublisher(GiskardBehavior):
    """
    Published tf for attached and environment objects.
    """

    @profile
    def __init__(self, name: str, mode: TfPublishingModes, tf_topic: str = 'tf', include_prefix: bool = True):
        super().__init__(name)
        self.original_links = set(GodMap.get_world().link_names_as_set)
        self.tf_pub = rospy.Publisher(tf_topic, TFMessage, queue_size=10)
        self.mode = mode
        self.robot_names = GodMap.get_collision_scene().robot_names
        self.include_prefix = include_prefix

    def make_transform(self, parent_frame, child_frame, pose):
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.header.stamp = rospy.get_rostime()
        tf.child_frame_id = child_frame
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = normalize_quaternion_msg(pose.orientation)
        return tf

    @record_time
    @profile
    def update(self):
        try:
            with GodMap.god_map as god_map:
                if self.mode == TfPublishingModes.all:
                    self.tf_pub.publish(GodMap.get_world().as_tf_msg(self.include_prefix))
                else:
                    tf_msg = TFMessage()
                    if self.mode in [TfPublishingModes.attached_objects, TfPublishingModes.attached_and_world_objects]:
                        for robot_name in self.robot_names:
                            robot_links = set(GodMap.get_world().groups[robot_name].link_names_as_set)
                        attached_links = robot_links - self.original_links
                        if attached_links:
                            get_fk = GodMap.get_world().compute_fk_pose
                            for link_name in attached_links:
                                parent_link_name = GodMap.get_world().get_parent_link_of_link(link_name)
                                fk = get_fk(parent_link_name, link_name)
                                if self.include_prefix:
                                    tf = self.make_transform(fk.header.frame_id, str(link_name), fk.pose)
                                else:
                                    tf = self.make_transform(fk.header.frame_id, str(link_name.short_name), fk.pose)
                                tf_msg.transforms.append(tf)
                if self.mode in [TfPublishingModes.world_objects, TfPublishingModes.attached_and_world_objects]:
                    for group_name, group in GodMap.get_world().groups.items():
                        if group_name in self.robot_names:
                            # robot frames will exist for sure
                            continue
                        if len(group.joints) > 0:
                            continue
                        get_fk = GodMap.get_world().compute_fk_pose
                        fk = get_fk(GodMap.get_world().root_link_name, group.root_link_name)
                        tf = self.make_transform(fk.header.frame_id, str(group.root_link_name), fk.pose)
                        tf_msg.transforms.append(tf)
                    self.tf_pub.publish(tf_msg)

        except KeyError as e:
            pass
        except UnboundLocalError as e:
            pass
        except ValueError as e:
            pass
        return Status.SUCCESS
