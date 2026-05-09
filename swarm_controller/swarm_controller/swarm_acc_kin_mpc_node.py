#!/usr/bin/env python3
"""Kinematic ACC MPC follower node (adas-style).

Subscribes to Telemetry, runs longitudinal kinematic MPC on a fixed timer,
publishes Twist. Acceleration command from MPC is integrated in-node into
v_cmd (strategy "B2") — this gives a smooth published velocity, analogous to
how the sliding-mode controller integrates `agent.v_ref`.

Like the Newton variant, the ROS node name is `swarm_controller` so that
`start.sh` works unchanged.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from swarm_msgs.msg import Telemetry

from .submodules.swarm_acc_kin_mpc import SwarmKinAccController


class SwarmAccKinMpcNode(Node):

    def __init__(self) -> None:
        super().__init__('swarm_controller')

        self.declare_parameters(namespace='', parameters=[
            # topics / id
            ('telemetry_topic', '/swarm_controller/telemetry'),
            ('cmd_vel_topic',   '/cmd_vel'),
            ('robot_id', -1),
            # plant (kinematic)
            ('tau', 0.2),
            # gap policy
            ('d0', 0.5),
            ('th', 0.0),
            # MPC tuning
            ('ts', 0.05),
            ('p', 20),
            ('c', 10),
            ('s', 3.0),
            # adas-style: 4 outputs [y_gap, v_rel, a, j]
            ('phi_vals', [0.6, 0.95, 0.6, 0.6]),
            ('q_vals',   [10.0, 1.0, 1.0, 1.0]),
            # constraints
            ('a_min', -0.5),
            ('a_max',  0.5),
            ('v_cmd_min', 0.0),
            ('v_cmd_max', 0.5),
            ('gap_safe',  0.2),
            # angular
            ('kp_theta', 0.3),
            # safety
            ('start', False),
            ('telemetry_timeout', 0.5),
        ])

        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.cmd_vel_topic   = self.get_parameter('cmd_vel_topic').value
        self.robot_id        = self.get_parameter('robot_id').value

        tau = self.get_parameter('tau').value
        d0 = self.get_parameter('d0').value
        th = self.get_parameter('th').value

        ts = self.get_parameter('ts').value
        p  = self.get_parameter('p').value
        c  = self.get_parameter('c').value
        s  = self.get_parameter('s').value
        phi_vals = list(self.get_parameter('phi_vals').value)
        q_vals   = list(self.get_parameter('q_vals').value)

        self.a_min = self.get_parameter('a_min').value
        self.a_max = self.get_parameter('a_max').value
        self.v_cmd_min = self.get_parameter('v_cmd_min').value
        self.v_cmd_max = self.get_parameter('v_cmd_max').value
        gap_safe = self.get_parameter('gap_safe').value

        self.kp_theta = float(self.get_parameter('kp_theta').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        self.ts = float(ts)

        self.get_logger().info(f'[swarm_acc_kin_mpc] robot_id={self.robot_id}')
        self.get_logger().info(f'  telemetry_topic: {self.telemetry_topic}')
        self.get_logger().info(f'  cmd_vel_topic:   {self.cmd_vel_topic}')
        self.get_logger().info(f'  plant: tau={tau}  (kinematic, adas-style)')
        self.get_logger().info(f'  gap policy: d0={d0}, th={th}')
        self.get_logger().info(f'  MPC: ts={ts}, p={p}, c={c}, s={s}')
        self.get_logger().info(f'  q_vals={q_vals}, phi_vals={phi_vals}')
        self.get_logger().info(
            f'  a_cmd in [{self.a_min}, {self.a_max}], v_cmd in [{self.v_cmd_min}, {self.v_cmd_max}]')
        self.get_logger().info(f'  gap_safe={gap_safe} (hard constraint)')

        self.controller = SwarmKinAccController(
            tau=tau,
            d0=d0, th=th,
            ts=ts, p=p, c=c, s=s,
            phi_vals=phi_vals, q_vals=q_vals,
            u_limits=(self.a_min, self.a_max),
            gap_safe=gap_safe,
        )

        self.last_telemetry = None
        self.last_telemetry_stamp = None
        self._was_started = False

        # B2 integrator: smooth v_cmd by integrating a_cmd in-node
        self._v_cmd_published = 0.0

        self._dbg_period = max(1, int(round(1.0 / ts)))
        self._dbg_counter = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            Telemetry, self.telemetry_topic, self._telemetry_cb, 10)

        self.create_timer(ts, self._control_step)
        self.get_logger().info('[swarm_acc_kin_mpc] ready, waiting for telemetry...')

    def _telemetry_cb(self, msg: Telemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_stamp = self.get_clock().now()

    def _control_step(self) -> None:
        started = bool(self.get_parameter('start').value)

        if started and not self._was_started:
            self.controller.reset()
            # initialise integrator from current measured v if telemetry available
            if self.last_telemetry is not None:
                self._v_cmd_published = float(self.last_telemetry.v)
            else:
                self._v_cmd_published = 0.0
            self.get_logger().info('[swarm_acc_kin_mpc] start → true, controller reset')
        self._was_started = started

        if not started:
            return

        if self.last_telemetry_stamp is None:
            return

        age = (self.get_clock().now() - self.last_telemetry_stamp).nanoseconds * 1e-9
        if age > self.telemetry_timeout:
            self.get_logger().warn(
                f'[swarm_acc_kin_mpc] telemetry stale ({age:.2f}s > {self.telemetry_timeout}s), stopping',
                throttle_duration_sec=1.0,
            )
            self._stop_robot()
            self.controller.reset()
            self._v_cmd_published = 0.0
            return

        msg = self.last_telemetry
        if not msg.is_valid:
            self._stop_robot()
            self.controller.reset()
            self._v_cmd_published = 0.0
            return

        # ── inputs ──────────────────────────────────────────────────────────
        dx_vec = np.array([msg.peer_x - msg.x, msg.peer_y - msg.y])
        dx = float(np.linalg.norm(dx_vec))
        v = float(msg.v)
        v_rel = float(msg.peer_v - msg.v)

        # ── MPC: returns acceleration command ───────────────────────────────
        a_cmd, y = self.controller.calculate_control(dx=dx, v=v, v_rel=v_rel)

        # ── B2 integrator: a_cmd → v_cmd ────────────────────────────────────
        self._v_cmd_published += self.ts * a_cmd
        self._v_cmd_published = float(np.clip(
            self._v_cmd_published, self.v_cmd_min, self.v_cmd_max))

        # ── angular control: rotate toward peer ─────────────────────────────
        az_global = float(np.arctan2(dx_vec[1], dx_vec[0]))
        az_rel = az_global - float(msg.theta)
        az_rel = float(np.arctan2(np.sin(az_rel), np.cos(az_rel)))
        w_cmd = self.kp_theta * az_rel

        # ── publish ─────────────────────────────────────────────────────────
        twist = Twist()
        twist.linear.x = self._v_cmd_published
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # throttled debug output
        self._dbg_counter += 1
        if self._dbg_counter >= self._dbg_period:
            self._dbg_counter = 0
            self.get_logger().info(
                f'dx={dx:.3f} v={v:.3f} v_rel={v_rel:+.3f}  '
                f'a_cmd={a_cmd:+.3f} v_cmd={self._v_cmd_published:.3f} w={w_cmd:+.3f}  '
                f'y_gap={y[0]:+.3f} a_pred={y[2]:+.3f} j_pred={y[3]:+.3f}',
            )

    def _stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmAccKinMpcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
