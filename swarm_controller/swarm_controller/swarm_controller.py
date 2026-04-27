#!/usr/bin/env python3


import os
import time

import numpy as np

import rclpy
from rclpy.node import Node

from swarm_msgs.msg import Telemetry
from geometry_msgs.msg import Twist

from .submodules.logs import write_log, init_log
from .submodules.swarm import Swarm
from .submodules.swarm_logic import *


class SwarmController(Node):

    def __init__(self) -> None:
        super().__init__('swarm_controller')

        self.declare_parameters(namespace='', parameters=[('telemetry_topic', ''),
                                                          ('logs_dir', ''),
                                                          ('robots_num', 0),
                                                          ('pacemaker_idx', 0),
                                                          ('robots_ids', [1, 2, 3]),
                                                          ('gap', 0.0),
                                                          ('v_max', 0.0),
                                                          ('acc_max', 0.0),
                                                          ('kp_theta', 0.0),
                                                          ('start', False),])
        
        telemetry_topic = self.get_parameter('telemetry_topic').value
        logs_dir = self.get_parameter('logs_dir').value
        self.robots_num = self.get_parameter('robots_num').value
        self.pacemaker_idx = self.get_parameter('pacemaker_idx').value
        self.robots_ids = self.get_parameter('robots_ids').value
        self.gap = self.get_parameter('gap').value
        self.v_max = np.abs(self.get_parameter('v_max').value)
        self.acc_max = self.get_parameter('acc_max').value
        self.kp_theta = self.get_parameter('kp_theta').value

        self.get_logger().info(f'telemetry_topic: {telemetry_topic}')
        self.get_logger().info(f'logs_dir: {logs_dir}')
        self.get_logger().info(f'robots_num: {self.robots_num}')
        self.get_logger().info(f'pacemaker_idx: {self.pacemaker_idx}')
        self.get_logger().info(f'robots_ids: {self.robots_ids}')
        self.get_logger().info(f'gap: {self.gap}')
        self.get_logger().info(f'v_max: {self.v_max}')
        self.get_logger().info(f'acc_max: {self.acc_max}')
        self.get_logger().info(f'kp_theta: {self.kp_theta}')
        
        self.log_files = dict()
        
        for robot_id in self.robots_ids:
            if robot_id == self.pacemaker_idx:
                continue
            self.log_files[robot_id] = os.path.join(logs_dir, f'agent_{robot_id}_{time.strftime("%Y_%m_%d_%H_%M_%S")}.duckdb')
            init_log(self.log_files[robot_id])

        self.create_subscription(Telemetry, telemetry_topic, self.telemetry_callback, 10)
        
        self.control_publishers = dict()
        self.dist_publishers = dict()
        self.t_prev = dict()
        
        for robot_id in self.robots_ids:
            control_pub = self.create_publisher(Twist, f'robot{robot_id}/cmd_vel', 10)
            
            self.control_publishers[robot_id] = control_pub
            self.t_prev[robot_id] = None
        
        self.swarm = Swarm(self.robots_num, self.robots_ids, self.pacemaker_idx)
        
        self.old_param_value = False
        
    def get_time(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds * 1e-9

    def stop_robots(self):
        for robot_id in self.robots_ids:
            if robot_id != self.pacemaker_idx:
                self.control_publishers[robot_id].publish(Twist())
        
    def telemetry_callback(self, msg : Telemetry):        
        if not msg.is_valid:
            self.stop_robots()
            return
        
        self.swarm.set_data_from_telemetry(msg, self.get_logger())

        for agent in self.swarm.agents:
            t = self.get_time()
        
            if self.t_prev[agent.id] is None:
                self.t_prev[agent.id] = t
                return
        
            if agent.id == self.pacemaker_idx:
                continue
            
            peers_vecs = get_peers_vecs(self.swarm.agents, agent, self.swarm.pacemaker, self.get_logger())           
            
            best_peers_vecs, azimuth = select_peers_and_azimuth(peers_vecs, self.swarm.pacemaker, agent, self.get_logger())
            
            a, w = control_cmd(best_peers_vecs, azimuth, agent, self.gap, self.acc_max, self.kp_theta, self.get_logger())
            
            dt = float(t - self.t_prev[agent.id])
            
            agent.v_ref += a * dt
            agent.w_ref = w

            agent.v_ref = np.clip(agent.v_ref, -self.v_max, self.v_max)
                
            twist = Twist()
            twist.linear.x = agent.v_ref
            twist.angular.z = agent.w_ref
                
            self.get_logger().info(f'agent {agent.id}: dt = {dt}, a = {a}, w = {twist.angular.z}, v = {twist.linear.x} is_valid: {msg.is_valid}')
            
            if self.get_parameter('start').value:
                if not self.old_param_value:
                    agent.v_ref = a * dt
                    self.old_param_value = True
                self.control_publishers[agent.id].publish(twist)
            
            self.t_prev[agent.id] = t
            
            write_log(self.log_files[agent.id], t, agent, peers_vecs, self.get_logger())
            

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
