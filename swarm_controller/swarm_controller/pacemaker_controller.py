#!/usr/bin/env python3

import os
from pathlib import Path as FilePath

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

from .submodules.swarm import Agent


def _resolve_trajectory_path(path_str: str) -> FilePath:
    """Resolve a trajectory yaml path.

    Absolute paths are used as-is. Relative paths and bare filenames are
    resolved against `swarm_controller/config/` — the package's config
    directory and where `trajectory_drawer` saves by default.

    The lookup follows the symlink chain `install → build → src` for a
    known existing file, so trajectories saved by `trajectory_drawer`
    into the source tree are visible immediately, without needing a
    fresh `colcon build`.
    """
    p = FilePath(path_str).expanduser()
    if p.is_absolute():
        return p
    share_config = FilePath(
        get_package_share_directory('swarm_controller')) / 'config'
    # Resolve to source config dir via realpath of a known stable file.
    for known in ('params_pacemaker.yaml', 'params_simulator.yaml',
                  'logging_topics.yaml'):
        anchor = share_config / known
        if anchor.exists():
            return FilePath(os.path.realpath(anchor)).parent / p
    return share_config / p  # fallback (no known anchor)


def _load_trajectory_file(path_str: str):
    """Resolve `trajectory_file` and read trajectory + waypoints from it.

    Returns (trajectory, waypoints_x, waypoints_y, resolved_path) — any
    of the first three may be None if the file does not specify them.
    Raises FileNotFoundError if the path does not exist.
    """
    p = _resolve_trajectory_path(path_str)
    if not p.exists():
        raise FileNotFoundError(f'trajectory_file not found: {p}')
    with open(p) as f:
        data = yaml.safe_load(f) or {}
    for top_val in data.values():
        if isinstance(top_val, dict) and 'ros__parameters' in top_val:
            params = top_val['ros__parameters']
            return (
                params.get('trajectory'),
                params.get('waypoints_x'),
                params.get('waypoints_y'),
                p,
            )
    raise ValueError(
        f'{p}: no /**: ros__parameters block found')


class PacemakerController(Node):

    def __init__(self) -> None:
        super().__init__('pacemaker_controller')

        self.declare_parameters(namespace='', parameters=[
            ('cmd_vel_topic', ''),
            ('pacemaker_idx', 0),
            ('start', False),
            ('trajectory', 'straight'),
            ('linear_vel', 0.3),
            ('circle_linear_vel', 0.3),
            ('circle_radius', 2.0),
            ('waypoints_x', [0.0]),
            ('waypoints_y', [0.0]),
            ('lookahead_dist', 1.0),
            ('trajectory_file', ''),
        ])

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pacemaker_idx = self.get_parameter('pacemaker_idx').value
        self.trajectory = self.get_parameter('trajectory').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.circle_linear_vel = self.get_parameter('circle_linear_vel').value
        self.circle_radius = self.get_parameter('circle_radius').value

        # If `trajectory_file` is set, load trajectory + waypoints from it and
        # override the values declared above.  This is the path written by
        # `trajectory_drawer` and the standard way to pick a custom route.
        traj_file_param = str(
            self.get_parameter('trajectory_file').value).strip()
        wx_override = wy_override = None
        if traj_file_param:
            traj, wx_override, wy_override, resolved = _load_trajectory_file(
                traj_file_param)
            if traj is not None:
                self.trajectory = traj
            self.get_logger().info(
                f'trajectory_file: {resolved} (trajectory={self.trajectory}, '
                f'{len(wx_override) if wx_override else 0} waypoints)')

        self.get_logger().info(f'cmd_vel_topic: {cmd_vel_topic}')
        self.get_logger().info(f'trajectory: {self.trajectory}')

        if self.trajectory == 'lanelet':
            if wx_override is not None and wy_override is not None:
                wx = list(wx_override)
                wy = list(wy_override)
            else:
                wx = list(self.get_parameter('waypoints_x').value)
                wy = list(self.get_parameter('waypoints_y').value)
            self.waypoints = np.array(list(zip(wx, wy)), dtype=float)
            self.lookahead_dist = self.get_parameter('lookahead_dist').value
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.path_pub = self.create_publisher(Path, '/pacemaker/path', 10)
            # Trajectory is "anchored" on the first valid pose: shifted AND
            # rotated so that (a) the first waypoint coincides with the
            # pacemaker's current (x, y) and (b) the initial tangent
            # (direction from wp[0] to wp[1]) coincides with the pacemaker's
            # current heading. So a trajectory drawn around the origin
            # facing +x in `trajectory_drawer` automatically picks up
            # the pacemaker's spawn pose, no manual offsetting needed.
            self._waypoints_anchored = False
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints, '
                f'lookahead={self.lookahead_dist} m '
                f'(start will be anchored to pacemaker pose)'
            )
        else:
            self.get_logger().info(f'linear_vel: {self.linear_vel}')
            self.get_logger().info(f'circle_linear_vel: {self.circle_linear_vel}')
            self.get_logger().info(f'circle_radius: {self.circle_radius}')

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.agent = Agent(self.pacemaker_idx)
        self.timer = self.create_timer(0.1, self.callback)

    def get_time(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds * 1e-9

    def _get_pose_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                f'robot{self.pacemaker_idx}/base_footprint',
                rclpy.time.Time(),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return float(t.x), float(t.y), float(theta)

    def _pure_pursuit(self, x, y, theta, v):
        pos = np.array([x, y])
        dists = np.linalg.norm(self.waypoints - pos, axis=1)
        closest_idx = int(np.argmin(dists))

        lookahead = None
        N = len(self.waypoints)
        for i in range(1, N + 1):
            idx = (closest_idx + i) % N
            if np.linalg.norm(self.waypoints[idx] - pos) >= self.lookahead_dist:
                lookahead = self.waypoints[idx]
                break

        if lookahead is None:
            lookahead = self.waypoints[closest_idx]

        dx = lookahead[0] - x
        dy = lookahead[1] - y
        alpha = np.arctan2(dy, dx) - theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        return 2.0 * v * np.sin(alpha) / self.lookahead_dist

    def _publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in self.waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def callback(self):
        t = self.get_time()

        if self.trajectory == 'straight':
            self.agent.v_ref = self.linear_vel
            self.agent.w_ref = 0.0
        elif self.trajectory == 'circle':
            self.agent.v_ref = self.circle_linear_vel
            self.agent.w_ref = self.circle_linear_vel / self.circle_radius
        elif self.trajectory == 'lanelet':
            pose = self._get_pose_from_tf()
            if pose is None:
                return
            x, y, theta = pose
            if not self._waypoints_anchored:
                wp0 = self.waypoints[0].copy()
                # Trajectory's initial heading = direction of first segment.
                if len(self.waypoints) >= 2:
                    seg = self.waypoints[1] - self.waypoints[0]
                    traj_theta = float(np.arctan2(seg[1], seg[0]))
                else:
                    traj_theta = 0.0
                dtheta = theta - traj_theta
                c, s = float(np.cos(dtheta)), float(np.sin(dtheta))
                R_T = np.array([[c, s], [-s, c]])  # row-vector rotation
                # Translate to origin, rotate, then translate to pacemaker pose.
                relative = self.waypoints - wp0
                self.waypoints = relative @ R_T + np.array([x, y])
                self._waypoints_anchored = True
                self.get_logger().info(
                    f'Trajectory anchored: shift=({x - wp0[0]:+.2f}, '
                    f'{y - wp0[1]:+.2f}) m, rotate={np.degrees(dtheta):+.1f}° '
                    f'→ start=({self.waypoints[0, 0]:.2f}, '
                    f'{self.waypoints[0, 1]:.2f}, '
                    f'θ={np.degrees(theta):+.1f}°)'
                )
            self.agent.v_ref = self.linear_vel
            self.agent.w_ref = self._pure_pursuit(x, y, theta, self.linear_vel)
            self._publish_path()
        else:
            self.get_logger().warn(
                f'unknown trajectory: {self.trajectory}',
                throttle_duration_sec=5.0,
            )
            self.agent.v_ref = 0.0
            self.agent.w_ref = 0.0

        twist = Twist()
        twist.linear.x = self.agent.v_ref
        twist.angular.z = self.agent.w_ref

        if self.get_parameter('start').value:
            self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f't={t:.2f} v={twist.linear.x:.3f} w={twist.angular.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = PacemakerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
