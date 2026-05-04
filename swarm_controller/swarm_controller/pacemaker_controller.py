#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

from .submodules.swarm import Agent


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
        ])

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pacemaker_idx = self.get_parameter('pacemaker_idx').value
        self.trajectory = self.get_parameter('trajectory').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.circle_linear_vel = self.get_parameter('circle_linear_vel').value
        self.circle_radius = self.get_parameter('circle_radius').value

        self.get_logger().info(f'cmd_vel_topic: {cmd_vel_topic}')
        self.get_logger().info(f'trajectory: {self.trajectory}')

        if self.trajectory == 'lanelet':
            wx = list(self.get_parameter('waypoints_x').value)
            wy = list(self.get_parameter('waypoints_y').value)
            self.waypoints = np.array(list(zip(wx, wy)), dtype=float)
            self.lookahead_dist = self.get_parameter('lookahead_dist').value
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.path_pub = self.create_publisher(Path, '/pacemaker/path', 10)
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints, lookahead={self.lookahead_dist} m'
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
