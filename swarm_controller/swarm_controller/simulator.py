#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation


class Simulator(Node):

    def __init__(self):
        super().__init__('simulator')

        self.declare_parameters(namespace='', parameters=[
            ('dt', 0.05),
            ('robots_ids', [1, 2]),
            ('pacemaker_idx', 1),
            ('noise_std', 0.02),
            ('init_x', [0.0, -1.0]),
            ('init_y', [0.0, 0.0]),
            ('init_theta', [0.0, 0.0]),
            # per-follower lists (order matches follower order in robots_ids)
            ('follower_peer_ids', [1]),          # which robot each follower observes
            ('follower_map_frames', ['robot2/map']),  # each follower's global_frame
            ('lidar_frames', ['lidar']),          # each follower's lidar frame name
            # shared lidar offset (same for all followers in simulation)
            ('lidar_offset_x', 0.0),
            ('lidar_offset_y', 0.0),
            ('lidar_offset_theta', 0.0),
            ('fov_min_radius', 0.2),
            ('fov_max_radius', 1.4),
        ])

        self.dt = self.get_parameter('dt').value
        self.robots_ids = list(self.get_parameter('robots_ids').value)
        self.pacemaker_idx = self.get_parameter('pacemaker_idx').value
        self.noise_std = self.get_parameter('noise_std').value
        init_x = list(self.get_parameter('init_x').value)
        init_y = list(self.get_parameter('init_y').value)
        init_theta = list(self.get_parameter('init_theta').value)
        follower_peer_ids = list(self.get_parameter('follower_peer_ids').value)
        follower_map_frames = list(self.get_parameter('follower_map_frames').value)
        lidar_frames = list(self.get_parameter('lidar_frames').value)
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.lidar_offset_y = self.get_parameter('lidar_offset_y').value
        self.lidar_offset_theta = self.get_parameter('lidar_offset_theta').value
        self.fov_min_radius = self.get_parameter('fov_min_radius').value
        self.fov_max_radius = self.get_parameter('fov_max_radius').value

        self.follower_ids = [rid for rid in self.robots_ids if rid != self.pacemaker_idx]

        # Per-follower lookup tables
        self.follower_peer_map = dict(zip(self.follower_ids, follower_peer_ids))
        self.follower_map_frame = dict(zip(self.follower_ids, follower_map_frames))
        self.lidar_frame_map = dict(zip(self.follower_ids, lidar_frames))

        # Robot state: {id: [x, y, theta, v, w]}
        self.states = {}
        for i, robot_id in enumerate(self.robots_ids):
            self.states[robot_id] = [
                float(init_x[i]),
                float(init_y[i]),
                float(init_theta[i]),
                0.0,
                0.0,
            ]

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        for robot_id in self.robots_ids:
            self.create_subscription(
                Twist,
                f'/robot{robot_id}/cmd_vel',
                lambda msg, rid=robot_id: self._cmd_vel_cb(msg, rid),
                10,
            )

        self.odom_pubs = {}
        self.scan_pubs = {}
        for robot_id in self.follower_ids:
            self.odom_pubs[robot_id] = self.create_publisher(
                Odometry, f'/robot{robot_id}/odom', 10)
            self.scan_pubs[robot_id] = self.create_publisher(
                LaserScan, f'/robot{robot_id}/scan', 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/simulator/markers', 10)

        self._publish_static_tfs()
        self.create_timer(self.dt, self._step)

        self.get_logger().info(f'Simulator started. Robots: {self.robots_ids}')
        for rid in self.follower_ids:
            peer = self.follower_peer_map[rid]
            self.get_logger().info(
                f'  robot{rid} follows robot{peer}'
                f' | map: {self.follower_map_frame[rid]}'
                f' | lidar: {self.lidar_frame_map[rid]}'
            )
        for rid, state in self.states.items():
            self.get_logger().info(
                f'  robot{rid}: x={state[0]:.2f}, y={state[1]:.2f}, theta={state[2]:.2f}')

    def _cmd_vel_cb(self, msg: Twist, robot_id: int):
        self.states[robot_id][3] = msg.linear.x
        self.states[robot_id][4] = msg.angular.z

    def _step(self):
        for robot_id, state in self.states.items():
            x, y, theta, v, w = state
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            theta = float(np.arctan2(np.sin(theta), np.cos(theta)))
            self.states[robot_id] = [x, y, theta, v, w]

        now = self.get_clock().now().to_msg()
        self._publish_tfs(now)
        for follower_id in self.follower_ids:
            self._publish_odom(follower_id, now)
            self._publish_scan(follower_id, now)
        self._publish_markers(now)

    # ── TF ──────────────────────────────────────────────────────────────────

    def _make_identity_tf(self, parent: str, child: str, stamp) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.rotation.w = 1.0
        return tf

    def _publish_static_tfs(self):
        stamp = self.get_clock().now().to_msg()
        tfs = []

        for follower_id in self.follower_ids:
            lidar_frame = self.lidar_frame_map[follower_id]
            map_frame = self.follower_map_frame[follower_id]

            # map → follower_N/map  (identity: simulation world = each robot's map)
            tfs.append(self._make_identity_tf('map', map_frame, stamp))

            # follower_N/base_footprint → lidar  (lidar offset, identity in simulation)
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = f'robot{follower_id}/base_footprint'
            tf.child_frame_id = lidar_frame
            tf.transform.translation.x = self.lidar_offset_x
            tf.transform.translation.y = self.lidar_offset_y
            tf.transform.translation.z = 0.0
            q = Rotation.from_euler('z', self.lidar_offset_theta).as_quat()
            tf.transform.rotation.x = float(q[0])
            tf.transform.rotation.y = float(q[1])
            tf.transform.rotation.z = float(q[2])
            tf.transform.rotation.w = float(q[3])
            tfs.append(tf)

        self.static_tf_broadcaster.sendTransform(tfs)

    def _publish_tfs(self, stamp):
        tfs = []
        for robot_id, state in self.states.items():
            x, y, theta = state[0], state[1], state[2]
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'map'
            tf.child_frame_id = f'robot{robot_id}/base_footprint'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            q = Rotation.from_euler('z', theta).as_quat()
            tf.transform.rotation.x = float(q[0])
            tf.transform.rotation.y = float(q[1])
            tf.transform.rotation.z = float(q[2])
            tf.transform.rotation.w = float(q[3])
            tfs.append(tf)
        self.tf_broadcaster.sendTransform(tfs)

    # ── Odometry ─────────────────────────────────────────────────────────────

    def _publish_odom(self, follower_id: int, stamp):
        x, y, theta, v, w = self.states[follower_id]
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.follower_map_frame[follower_id]
        odom.child_frame_id = f'robot{follower_id}/base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        q = Rotation.from_euler('z', theta).as_quat()
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pubs[follower_id].publish(odom)

    # ── LaserScan ────────────────────────────────────────────────────────────

    def _publish_scan(self, follower_id: int, stamp):
        peer_id = self.follower_peer_map[follower_id]
        f = self.states[follower_id]
        p = self.states[peer_id]

        # Peer position in follower body frame
        dx = p[0] - f[0]
        dy = p[1] - f[1]
        theta = f[2]
        bf_x = dx * np.cos(theta) + dy * np.sin(theta)
        bf_y = -dx * np.sin(theta) + dy * np.cos(theta)

        # Body frame → lidar frame (inverse of lidar offset)
        lx = self.lidar_offset_x
        ly = self.lidar_offset_y
        lt = self.lidar_offset_theta
        lidar_x = (bf_x - lx) * np.cos(lt) + (bf_y - ly) * np.sin(lt)
        lidar_y = -(bf_x - lx) * np.sin(lt) + (bf_y - ly) * np.cos(lt)

        r = float(np.sqrt(lidar_x**2 + lidar_y**2))
        phi = float(np.arctan2(lidar_y, lidar_x))

        n_beams = 360
        angle_min = -np.pi
        angle_increment = 2 * np.pi / n_beams

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.lidar_frame_map[follower_id]
        scan.angle_min = angle_min
        scan.angle_max = float(angle_min + (n_beams - 1) * angle_increment)
        scan.angle_increment = angle_increment
        scan.range_min = 0.05
        scan.range_max = 20.0
        scan.ranges = [float('inf')] * n_beams

        if self.fov_min_radius <= r <= self.fov_max_radius:
            n_pts = 10
            for i in range(n_pts):
                angle_off = (i - n_pts / 2) * 0.015
                beam_idx = int(round((phi + angle_off - angle_min) / angle_increment)) % n_beams
                scan.ranges[beam_idx] = float(r + np.random.normal(0.0, self.noise_std))
        else:
            self.get_logger().warn(
                f'robot{follower_id}: robot{peer_id} out of FOV '
                f'r={r:.2f} phi={np.degrees(phi):.1f}°',
                throttle_duration_sec=1.0,
            )

        self.scan_pubs[follower_id].publish(scan)

    # ── Markers ──────────────────────────────────────────────────────────────

    def _publish_markers(self, stamp):
        markers = MarkerArray()

        for robot_id, state in self.states.items():
            x, y, theta, _, _ = state
            is_pacemaker = (robot_id == self.pacemaker_idx)
            q = Rotation.from_euler('z', theta).as_quat()

            body = Marker()
            body.header.stamp = stamp
            body.header.frame_id = 'map'
            body.ns = 'robots'
            body.id = robot_id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = 0.1
            body.pose.orientation.x = float(q[0])
            body.pose.orientation.y = float(q[1])
            body.pose.orientation.z = float(q[2])
            body.pose.orientation.w = float(q[3])
            body.scale.x = 0.3
            body.scale.y = 0.3
            body.scale.z = 0.2
            body.color.a = 1.0
            body.color.r = 1.0 if is_pacemaker else 0.2
            body.color.g = 0.2 if is_pacemaker else 0.4
            body.color.b = 0.0 if is_pacemaker else 1.0
            markers.markers.append(body)

            arrow = Marker()
            arrow.header.stamp = stamp
            arrow.header.frame_id = 'map'
            arrow.ns = 'heading'
            arrow.id = robot_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.2
            arrow.pose.orientation.x = float(q[0])
            arrow.pose.orientation.y = float(q[1])
            arrow.pose.orientation.z = float(q[2])
            arrow.pose.orientation.w = float(q[3])
            arrow.scale.x = 0.5
            arrow.scale.y = 0.07
            arrow.scale.z = 0.07
            arrow.color.a = 1.0
            arrow.color.r = 1.0 if is_pacemaker else 0.0
            arrow.color.g = 0.5 if is_pacemaker else 0.9
            arrow.color.b = 0.0
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
