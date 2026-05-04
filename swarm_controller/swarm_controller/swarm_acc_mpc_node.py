#!/usr/bin/env python3
"""ACC MPC follower node — replacement for sliding-mode `swarm_controller.py`.

Subscribes to Telemetry, runs longitudinal MPC on a fixed timer, publishes Twist.
Angular velocity is set by a simple geometric controller pointing at the peer.

This node is intended to replace the Python sliding-mode controller when the
parameter `sliding_mode: false` is set in the YAML. To allow `start.sh` to keep
working unchanged, the ROS node name is `swarm_controller` (same as the
sliding-mode counterpart).
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from swarm_msgs.msg import Telemetry

from .submodules.swarm_acc_mpc import SwarmAccController


class SwarmAccMpcNode(Node):

    def __init__(self) -> None:
        # node name kept as swarm_controller for compatibility with start.sh
        super().__init__('swarm_controller')

        self.declare_parameters(namespace='', parameters=[
            # topics / id
            ('telemetry_topic', '/swarm_controller/telemetry'),
            ('cmd_vel_topic', '/cmd_vel'),
            ('robot_id', -1),
            # plant (Newton + force lag)
            ('m', 2.0),
            ('b', 1.0),
            ('alpha', 4.0),
            ('tau_F', 0.2),
            # gap policy
            ('d0', 0.5),
            ('th', 0.0),
            # MPC tuning
            ('ts', 0.05),
            ('p', 20),
            ('c', 10),
            ('s', 3.0),
            # length 4: [y_gap, v_rel, F, e_int]
            # PI-D-like cost with anti-oscillation tuning — see yaml
            ('phi_vals', [0.6, 0.95, 0.6, 0.9]),
            ('q_vals', [10.0, 1.0, 0.0, 1.0]),
            # constraints — input is a_cmd (acceleration), not v_cmd
            ('a_min', -0.5),
            ('a_max',  0.5),
            ('v_cmd_min', 0.0),
            ('v_cmd_max', 0.5),
            ('F_min', -5.0),
            ('F_max', 5.0),
            ('gap_safe', 0.3),    # HARD safety constraint: dx ≥ gap_safe
            # angular
            ('kp_theta', 0.3),
            # safety
            ('start', False),
            ('telemetry_timeout', 0.5),
        ])

        # topics
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.robot_id = self.get_parameter('robot_id').value

        # plant
        m = self.get_parameter('m').value
        b = self.get_parameter('b').value
        alpha = self.get_parameter('alpha').value
        tau_F = self.get_parameter('tau_F').value

        # policy
        d0 = self.get_parameter('d0').value
        th = self.get_parameter('th').value

        # MPC
        ts = self.get_parameter('ts').value
        p = self.get_parameter('p').value
        c = self.get_parameter('c').value
        s = self.get_parameter('s').value
        phi_vals = list(self.get_parameter('phi_vals').value)
        q_vals = list(self.get_parameter('q_vals').value)

        # constraints
        a_min = self.get_parameter('a_min').value
        a_max = self.get_parameter('a_max').value
        v_cmd_min = self.get_parameter('v_cmd_min').value
        v_cmd_max = self.get_parameter('v_cmd_max').value
        F_min = self.get_parameter('F_min').value
        F_max = self.get_parameter('F_max').value
        gap_safe = self.get_parameter('gap_safe').value

        self.kp_theta = float(self.get_parameter('kp_theta').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)

        # log key parameters
        self.get_logger().info(f'[swarm_acc_mpc] robot_id={self.robot_id}')
        self.get_logger().info(f'  telemetry_topic: {self.telemetry_topic}')
        self.get_logger().info(f'  cmd_vel_topic:   {self.cmd_vel_topic}')
        self.get_logger().info(
            f'  plant: m={m}, b={b}, alpha={alpha}, tau_F={tau_F}')
        self.get_logger().info(f'  gap policy: d0={d0}, th={th}')
        self.get_logger().info(f'  MPC: ts={ts}, p={p}, c={c}, s={s}')
        self.get_logger().info(f'  q_vals={q_vals}, phi_vals={phi_vals}')
        self.get_logger().info(
            f'  a_cmd in [{a_min}, {a_max}], v_cmd in [{v_cmd_min}, {v_cmd_max}]')
        self.get_logger().info(f'  F in [{F_min}, {F_max}], kp_theta={self.kp_theta}')
        self.get_logger().info(f'  gap_safe={gap_safe} (hard collision-avoidance)')

        # build controller (validates param dimensions)
        self.controller = SwarmAccController(
            m=m, b=b, alpha=alpha, tau_F=tau_F,
            d0=d0, th=th,
            ts=ts, p=p, c=c, s=s,
            phi_vals=phi_vals, q_vals=q_vals,
            a_limits=(a_min, a_max),
            v_cmd_limits=(v_cmd_min, v_cmd_max),
            F_limits=(F_min, F_max),
            gap_safe=gap_safe,
        )

        # state
        self.last_telemetry = None
        self.last_telemetry_stamp = None
        self._was_started = False

        # debug log throttling: emit one info-line per second
        self._dbg_period = max(1, int(round(1.0 / ts)))
        self._dbg_counter = 0

        # publisher / subscription
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            Telemetry, self.telemetry_topic, self._telemetry_cb, 10)

        # control timer
        self.create_timer(ts, self._control_step)

        self.get_logger().info('[swarm_acc_mpc] ready, waiting for telemetry...')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _telemetry_cb(self, msg: Telemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_stamp = self.get_clock().now()

    def _control_step(self) -> None:
        started = bool(self.get_parameter('start').value)

        # transition false → true: clear stale predictor state
        if started and not self._was_started:
            self.controller.reset()
            self.get_logger().info('[swarm_acc_mpc] start → true, controller reset')
        self._was_started = started

        if not started:
            return

        # nothing arrived yet
        if self.last_telemetry_stamp is None:
            return

        # stale telemetry → safe stop
        age = (self.get_clock().now() - self.last_telemetry_stamp).nanoseconds * 1e-9
        if age > self.telemetry_timeout:
            self.get_logger().warn(
                f'[swarm_acc_mpc] telemetry stale ({age:.2f}s > {self.telemetry_timeout}s), stopping',
                throttle_duration_sec=1.0,
            )
            self._stop_robot()
            self.controller.reset()
            return

        msg = self.last_telemetry
        if not msg.is_valid:
            self._stop_robot()
            self.controller.reset()
            return

        # ── inputs ──────────────────────────────────────────────────────────
        dx_vec = np.array([msg.peer_x - msg.x, msg.peer_y - msg.y])
        dx = float(np.linalg.norm(dx_vec))
        v = float(msg.v)
        v_rel = float(msg.peer_v - msg.v)

        # ── longitudinal MPC ────────────────────────────────────────────────
        # MPC returns (v_cmd_published, a_cmd_optimal, output_y).
        # v_cmd is the integrator state — smooth by construction.
        v_cmd, a_cmd, y = self.controller.calculate_control(dx=dx, v=v, v_rel=v_rel)

        # ── geometric angular control: rotate toward peer ───────────────────
        az_global = float(np.arctan2(dx_vec[1], dx_vec[0]))
        az_rel = az_global - float(msg.theta)
        az_rel = float(np.arctan2(np.sin(az_rel), np.cos(az_rel)))  # wrap to [-π, π]
        w_cmd = self.kp_theta * az_rel

        # ── publish ─────────────────────────────────────────────────────────
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # throttled debug output
        self._dbg_counter += 1
        if self._dbg_counter >= self._dbg_period:
            self._dbg_counter = 0
            self.get_logger().info(
                f'dx={dx:.3f} v={v:.3f} v_rel={v_rel:+.3f}  '
                f'a_cmd={a_cmd:+.3f} v_cmd={v_cmd:.3f} w={w_cmd:+.3f}  '
                f'y_gap={y[0]:+.3f} F_pred={y[2]:+.3f}',
            )

    # ── helpers ──────────────────────────────────────────────────────────────

    def _stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmAccMpcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
