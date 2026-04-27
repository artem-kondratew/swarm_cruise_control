#!/usr/bin/env python3

import os
import time

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from .submodules.logs import write_log, init_log
from .submodules.swarm import Agent


class PacemakerController(Node):

    def __init__(self) -> None:
        super().__init__('pacemaker_controller')

        self.declare_parameters(namespace='', parameters=[
            ('cmd_vel_topic', ''),
            ('pacemaker_idx', 0),
            ('logs_dir', ''),
            ('start', False),
            ('trajectory', 'straight'),
            ('linear_vel', 0.3),
            ('circle_linear_vel', 0.3),
            ('circle_radius', 2.0),
        ])

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pacemaker_idx = self.get_parameter('pacemaker_idx').value
        logs_dir = self.get_parameter('logs_dir').value
        self.trajectory = self.get_parameter('trajectory').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.circle_linear_vel = self.get_parameter('circle_linear_vel').value
        self.circle_radius = self.get_parameter('circle_radius').value

        self.get_logger().info(f'cmd_vel_topic: {cmd_vel_topic}')
        self.get_logger().info(f'trajectory: {self.trajectory}')
        self.get_logger().info(f'linear_vel: {self.linear_vel}')
        self.get_logger().info(f'circle_linear_vel: {self.circle_linear_vel}')
        self.get_logger().info(f'circle_radius: {self.circle_radius}')

        self.log_file = os.path.join(
            logs_dir,
            f'agent_{self.pacemaker_idx}_{time.strftime("%Y_%m_%d_%H_%M_%S")}.duckdb',
        )
        init_log(self.log_file)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.agent = Agent(self.pacemaker_idx)
        self.timer = self.create_timer(0.1, self.callback)

    def get_time(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds * 1e-9

    def callback(self):
        t = self.get_time()

        if self.trajectory == 'straight':
            self.agent.v_ref = self.linear_vel
            self.agent.w_ref = 0.0
        elif self.trajectory == 'circle':
            self.agent.v_ref = self.circle_linear_vel
            self.agent.w_ref = self.circle_linear_vel / self.circle_radius
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

        write_log(self.log_file, t, self.agent, [], self.get_logger())


def main(args=None):
    rclpy.init(args=args)
    node = PacemakerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
