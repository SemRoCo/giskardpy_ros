from copy import deepcopy
from typing import Optional, List, Dict, Union

import numpy as np
from geometry_msgs.msg import Vector3, Point, Quaternion
from line_profiler import profile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

import giskardpy.casadi_wrapper as cas
import giskardpy_ros.ros2.msg_converter as msg_converter
from giskardpy.data_types.data_types import PrefixName
from giskardpy.god_map import god_map
from giskardpy.model.collision_world_syncer import Collisions
from giskardpy.model.links import Link
from giskardpy.model.trajectory import Trajectory
from giskardpy.utils.decorators import clear_memo, memoize
from giskardpy.utils.math import rotation_matrix_from_axis_angle, quaternion_from_rotation_matrix
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard


class ROSMsgVisualization:
    red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    yellow = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
    green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    colors = [
        red,  # red
        ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # blue
        yellow,  # yellow
        ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),  # violet
        ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),  # cyan
        green,  # green
        ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),  # white
        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # black
    ]
    mode: VisualizationMode
    frame_locked: bool
    world_version: int


    def __init__(self, tf_frame: Optional[str] = None,
                 visualization_topic: str = '~visualization_marker_array',
                 scale_scale: float = 1.0,
                 mode: VisualizationMode = VisualizationMode.CollisionsDecomposed):
        self.mode = mode
        self.scale_scale = scale_scale
        self.frame_locked = self.mode in [VisualizationMode.VisualsFrameLocked,
                                          VisualizationMode.CollisionsFrameLocked,
                                          VisualizationMode.CollisionsDecomposedFrameLocked]
        # qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = rospy.node.create_publisher(MarkerArray,
                                                   f'{rospy.node.get_name()}/visualization_marker_array',
                                               10)
        self.publisher_aux = rospy.node.create_publisher(MarkerArray,
                                                   f'{rospy.node.get_name()}/visualization_marker_array/aux',
                                               10)
        self.marker_ids = {}
        if tf_frame is None:
            self.tf_root = str(god_map.world.root_link_name)
        else:
            self.tf_root = tf_frame
        GiskardBlackboard().ros_visualizer = self
        self.world_version = -1

    @memoize
    def link_to_marker(self, link: Link) -> List[Marker]:
        ms = msg_converter.link_to_visualization_marker(data=link, mode=self.mode).markers
        for m in ms:
            m.scale.x *= self.scale_scale
            m.scale.y *= self.scale_scale
            m.scale.z *= self.scale_scale
        return ms

    def clear_marker_cache(self) -> None:
        clear_memo(self.link_to_marker)

    def has_world_changed(self) -> bool:
        if self.world_version != god_map.world.model_version:
            self.world_version = god_map.world.model_version
            return True
        return False


    def create_world_markers(self, name_space: str = 'world', marker_id_offset: int = 0) -> List[Marker]:
        markers = []
        time_stamp = rospy.node.get_clock().now().to_msg()
        if self.mode in [VisualizationMode.Visuals, VisualizationMode.VisualsFrameLocked]:
            links = god_map.world.link_names
        else:
            links = god_map.world.link_names_with_collisions
        for i, link_name in enumerate(links):
            link = god_map.world.links[link_name]
            collision_markers = self.link_to_marker(link)
            for j, marker in enumerate(collision_markers):
                if self.frame_locked:
                    marker.header.frame_id = link_name.short_name
                else:
                    marker.header.frame_id = self.tf_root
                marker.action = Marker.ADD
                link_id_key = f'{link_name}_{j}'
                if link_id_key not in self.marker_ids:
                    self.marker_ids[link_id_key] = len(self.marker_ids)
                marker.id = self.marker_ids[link_id_key] + marker_id_offset
                marker.ns = name_space
                marker.header.stamp = time_stamp
                if self.frame_locked:
                    marker.frame_locked = True
                else:
                    marker.pose = god_map.collision_scene.get_map_T_geometry(link_name, j)
                markers.append(marker)
        return markers


    def create_collision_markers(self, name_space: str = 'collisions') -> List[Marker]:
        try:
            collisions: Collisions = god_map.closest_point
        except AttributeError as e:
            # no collisions
            return []
        collision_avoidance_configs = god_map.collision_scene.collision_avoidance_configs
        m = Marker()
        m.header.frame_id = self.tf_root
        m.action = Marker.ADD
        m.type = Marker.LINE_LIST
        m.id = 1337
        m.ns = name_space
        m.scale = Vector3(x=0.003, y=0.0, z=0.0)
        m.pose.orientation.w = 1.0
        if len(collisions.all_collisions) > 0:
            for collision in collisions.all_collisions:
                group_name = collision.link_a.prefix
                config = collision_avoidance_configs[group_name]
                if collision.is_external:
                    thresholds = config.external_collision_avoidance[collision.link_a]
                else:
                    thresholds = config.self_collision_avoidance[collision.link_a]
                red_threshold = thresholds.hard_threshold
                yellow_threshold = thresholds.soft_threshold
                contact_distance = collision.contact_distance
                if collision.map_P_pa is None:
                    map_T_a = god_map.world.compute_fk_np(god_map.world.root_link_name, collision.original_link_a)
                    map_P_pa = np.dot(map_T_a, collision.a_P_pa)
                else:
                    map_P_pa = collision.map_P_pa

                if collision.map_P_pb is None:
                    map_T_b = god_map.world.compute_fk_np(god_map.world.root_link_name, collision.original_link_b)
                    map_P_pb = np.dot(map_T_b, collision.b_P_pb)
                else:
                    map_P_pb = collision.map_P_pb
                m.points.append(Point(x=map_P_pa[0], y=map_P_pa[1], z=map_P_pa[2]))
                m.points.append(Point(x=map_P_pb[0], y=map_P_pb[1], z=map_P_pb[2]))
                m.colors.append(self.red)
                m.colors.append(self.green)
                if contact_distance < yellow_threshold:
                    # m.colors[-2] = self.yellow
                    m.colors[-1] = self.yellow
                if contact_distance < red_threshold:
                    # m.colors[-2] = self.red
                    m.colors[-1] = self.red
        else:
            return []
        return [m]


    def publish_markers(self, world_ns: str = 'world', collision_ns: str = 'collisions', force: bool = False) -> None:
        if not self.mode == VisualizationMode.Nothing:
            marker_array = MarkerArray()
            if force or (not self.frame_locked or self.frame_locked and self.has_world_changed()):
                # self.clear_marker(world_ns)
                marker_array.markers.extend(self.create_world_markers(name_space=world_ns))
            marker_array.markers.extend(self.create_collision_markers(name_space=collision_ns))
            if len(marker_array.markers) > 0:
                self.publisher.publish(marker_array)

    def publish_trajectory_markers(self, trajectory: Trajectory, every_x: int = 10,
                                   start_alpha: float = 0.5, stop_alpha: float = 1.0,
                                   namespace: str = 'trajectory') -> None:
        self.clear_marker(namespace)
        marker_array = MarkerArray()

        def compute_alpha(i):
            if i < 0 or i >= len(trajectory):
                raise ValueError(f'Index {i} is out of range {len(trajectory)}')
            return start_alpha + i * (stop_alpha - start_alpha) / (len(trajectory) - 1)

        with god_map.world.reset_joint_state_context():
            for point_id, joint_state in trajectory.items():
                if point_id % every_x == 0 or point_id == len(trajectory) - 1:
                    god_map.world.state = joint_state
                    god_map.world.notify_state_change()
                    if self.mode not in [VisualizationMode.Visuals, VisualizationMode.VisualsFrameLocked]:
                        god_map.collision_scene.sync()
                    markers = self.create_world_markers(name_space=namespace,
                                                        marker_id_offset=len(marker_array.markers))
                    for m in markers:
                        m.color.a = compute_alpha(point_id)
                    marker_array.markers.extend(deepcopy(markers))
        self.publisher.publish(marker_array)

    def publish_debug_trajectory(self,
                                 debug_expressions: Dict[PrefixName, Union[cas.TransMatrix,
                                 cas.Point3,
                                 cas.Vector3,
                                 cas.Quaternion]],
                                 raw_debug_trajectory: List[Dict[PrefixName, np.ndarray]],
                                 joint_space_traj: Trajectory,
                                 every_x: int = 10,
                                 start_alpha: float = 0.15, stop_alpha: float = 1.0,
                                 namespace: str = 'debug_trajectory') -> None:
        self.clear_marker(namespace)
        marker_array = MarkerArray()

        def compute_alpha(i):
            if i < 0 or i >= len(raw_debug_trajectory):
                raise ValueError("Index i is out of range")
            return start_alpha + i * (stop_alpha - start_alpha) / (len(raw_debug_trajectory) - 1)

        def scale_color_to_white(original_color: ColorRGBA, scale: float) -> ColorRGBA:
            """
            Scales the input RGBA color between white and the original color based on the scale value.

            :param original_color: The original ColorRGBA message.
            :param scale: A value between 0 and 1, where 0 is white and 1 is the original color.
            :return: A new ColorRGBA message scaled between white and the original color.
            """
            scale = max(0.0, min(1.0, scale))  # Ensure scale is clamped between 0 and 1

            new_color = ColorRGBA()
            new_color.r = (1.0 - scale) + scale * original_color.r
            new_color.g = (1.0 - scale) + scale * original_color.g
            new_color.b = (1.0 - scale) + scale * original_color.b
            new_color.a = original_color.a  # Keep alpha unchanged

            return new_color

        with god_map.world.reset_joint_state_context():
            for point_id, point in enumerate(raw_debug_trajectory):
                joint_state = joint_space_traj.get_exact(point_id)
                god_map.world.state = joint_state
                god_map.world.notify_state_change()
                if self.mode not in [VisualizationMode.Visuals, VisualizationMode.VisualsFrameLocked]:
                    god_map.collision_scene.sync()
                if point_id % every_x == 0 or point_id == len(raw_debug_trajectory) - 1:
                    markers = self.debug_state_to_vectors_markers(debug_expressions=debug_expressions,
                                                                  debug_values=point,
                                                                  marker_id_offset=len(marker_array.markers))
                    for m in markers:
                        m.color = scale_color_to_white(m.color, start_alpha + (point_id+1)/len(raw_debug_trajectory))
                        # m.color.r = min(1-m.color.r+compute_alpha(point_id), 1)
                        # m.color.g = min(1-m.color.g+compute_alpha(point_id), 1)
                        # m.color.b = min(1-m.color.b+compute_alpha(point_id), 1)
                        # m.color.a = 1
                    marker_array.markers.extend(deepcopy(markers))
        self.publisher.publish(marker_array)

    def clear_marker(self, ns: str):
        msg = MarkerArray()
        for i in self.marker_ids.values():
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = i
            marker.ns = ns
            msg.markers.append(marker)
        self.publisher.publish(msg)
        self.marker_ids = {}

    def debug_state_to_vectors_markers(self,
                                       debug_expressions: Dict[PrefixName, Union[cas.TransMatrix,
                                       cas.Point3,
                                       cas.Vector3,
                                       cas.Quaternion,
                                       cas.RotationMatrix]],
                                       debug_values: Dict[PrefixName, np.ndarray],
                                       width: float = 0.05,
                                       marker_id_offset: int = 0) -> List[Marker]:
        ms = []
        color_counter = 0
        for (name, expr), (_, value) in zip(debug_expressions.items(), debug_values.items()):
            if not hasattr(expr, 'reference_frame'):
                continue
            if expr.reference_frame is not None:
                map_T_ref = god_map.world.compute_fk_np(god_map.world.root_link_name, expr.reference_frame)
            else:
                map_T_ref = np.eye(4)

            if isinstance(expr, cas.RotationMatrix):
                colors = [
                    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red (X)
                    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Green (Y)
                    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue (Z)
                ]

                for i in range(3):
                    m = Marker()
                    m.header.frame_id = self.tf_root
                    m.header.stamp = rospy.node.get_clock().now().to_msg()
                    m.pose.orientation.w = 1.0
                    m.ns = f'debug/{name}'
                    m.id = i + marker_id_offset
                    m.type = Marker.ARROW
                    m.action = Marker.ADD
                    axis = value[:, i] * 0.5
                    m.points = [Point(), Point(x=axis[0], y=axis[1], z=axis[2])]  # Start and Endpoints
                    # Arrow properties
                    m.scale.x = width / 2.
                    m.scale.y = width
                    m.scale.z = 0.

                    m.color = colors[i]

                    ms.append(m)
            elif isinstance(expr, cas.TransMatrix):
                ref_T_d = value
                map_T_d = np.dot(map_T_ref, ref_T_d)
                map_P_d = map_T_d[:4, 3:]
                # x
                d_V_x_offset = np.array([width, 0., 0., 0.])
                map_V_x_offset = np.dot(map_T_d, d_V_x_offset)
                mx = Marker()
                mx.action = Marker.ADD
                mx.header.frame_id = self.tf_root
                mx.ns = f'debug/{name}'
                mx.id = 0 + marker_id_offset
                mx.type = Marker.CYLINDER
                mx.pose.position.x = map_P_d[0][0] + map_V_x_offset[0]
                mx.pose.position.y = map_P_d[1][0] + map_V_x_offset[1]
                mx.pose.position.z = map_P_d[2][0] + map_V_x_offset[2]
                d_R_x = rotation_matrix_from_axis_angle([0, 1, 0], np.pi / 2)
                map_R_x = np.dot(map_T_d, d_R_x)
                q = quaternion_from_rotation_matrix(map_R_x)
                mx.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                mx.color = ColorRGBA(r=1., g=0., b=0., a=1.)
                mx.scale.x = width / 4.
                mx.scale.y = width / 4.
                mx.scale.z = width * 2.
                ms.append(mx)
                # y
                d_V_y_offset = np.array([0., width, 0., 0.])
                map_V_y_offset = np.dot(map_T_d, d_V_y_offset)
                my = Marker()
                my.action = Marker.ADD
                my.header.frame_id = self.tf_root
                my.ns = f'debug/{name}'
                my.id = 1 + marker_id_offset
                my.type = Marker.CYLINDER
                my.pose.position.x = map_P_d[0][0] + map_V_y_offset[0]
                my.pose.position.y = map_P_d[1][0] + map_V_y_offset[1]
                my.pose.position.z = map_P_d[2][0] + map_V_y_offset[2]
                d_R_y = rotation_matrix_from_axis_angle([1., 0, 0], -np.pi / 2)
                map_R_y = np.dot(map_T_d, d_R_y)
                q = quaternion_from_rotation_matrix(map_R_y)
                my.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                my.color = ColorRGBA(r=0., g=1., b=0., a=1.)
                my.scale.x = width / 4.
                my.scale.y = width / 4.
                my.scale.z = width * 2.
                ms.append(my)
                # z
                d_V_z_offset = np.array([0, 0, width, 0])
                map_V_z_offset = np.dot(map_T_d, d_V_z_offset)
                mz = Marker()
                mz.action = Marker.ADD
                mz.header.frame_id = self.tf_root
                mz.ns = f'debug/{name}'
                mz.id = 2 + marker_id_offset
                mz.type = Marker.CYLINDER
                mz.pose.position.x = map_P_d[0][0] + map_V_z_offset[0]
                mz.pose.position.y = map_P_d[1][0] + map_V_z_offset[1]
                mz.pose.position.z = map_P_d[2][0] + map_V_z_offset[2]
                q = quaternion_from_rotation_matrix(map_T_d)
                mz.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                mz.color = ColorRGBA(r=0., g=0., b=1., a=1.)
                mz.scale.x = width / 4.
                mz.scale.y = width / 4.
                mz.scale.z = width * 2.
                ms.append(mz)
            else:
                m = Marker()
                m.action = Marker.ADD
                m.ns = f'debug/{name}'
                m.id = 0 + marker_id_offset
                m.header.frame_id = self.tf_root
                m.pose.orientation.w = 1.
                if isinstance(expr, cas.Vector3):
                    ref_V_d = value
                    if expr.vis_frame is not None:
                        map_T_vis = god_map.world.compute_fk_np(god_map.world.root_link_name, expr.vis_frame)
                    else:
                        map_T_vis = np.eye(4)
                    map_V_d = np.dot(map_T_ref, ref_V_d)
                    map_P_vis = map_T_vis[:4, 3:].T[0]
                    map_P_p1 = map_P_vis
                    map_P_p2 = map_P_vis + map_V_d
                    m.points.append(Point(x=map_P_p1[0], y=map_P_p1[1], z=map_P_p1[2]))
                    m.points.append(Point(x=map_P_p2[0], y=map_P_p2[1], z=map_P_p2[2]))
                    m.type = Marker.ARROW
                    if expr.color is None:
                        m.color = self.colors[color_counter]
                    else:
                        m.color = ColorRGBA(r=expr.color.r, g=expr.color.g, b=expr.color.b, a=expr.color.a)
                    m.scale.x = width / 2.
                    m.scale.y = width
                    m.scale.z = 0.
                    color_counter += 1
                elif isinstance(expr, cas.Point3):
                    ref_P_d = value
                    map_P_d = np.dot(map_T_ref, ref_P_d)
                    m.pose.position.x = map_P_d[0]
                    m.pose.position.y = map_P_d[1]
                    m.pose.position.z = map_P_d[2]
                    m.pose.orientation.w = 1.
                    m.type = Marker.SPHERE
                    if expr.color is None:
                        m.color = self.colors[color_counter]
                    else:
                        m.color = ColorRGBA(r=expr.color.r, g=expr.color.g, b=expr.color.b, a=expr.color.a)
                    m.scale.x = width
                    m.scale.y = width
                    m.scale.z = width
                    color_counter += 1
                ms.append(m)
        return ms

    def pub_box_marker(self, name: str, frame_id: str, xyz: List[float], color: ColorRGBA) -> None:
        m = Marker()
        m.scale.x = xyz[0]
        m.scale.y = xyz[1]
        m.scale.z = xyz[2]
        m.frame_locked = True
        m.header.frame_id = frame_id
        m.ns = name
        m.id = 3213
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.color = color
        m.pose.orientation.w = 1.0
        marker_array = MarkerArray()
        marker_array.markers.append(m)
        self.publisher_aux.publish(marker_array)
