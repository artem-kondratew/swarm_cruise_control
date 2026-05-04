#!/usr/bin/env python3

import numpy as np

import rclpy
import tf2_ros

from scipy.spatial.transform import Rotation
from sklearn.cluster import KMeans

from rclpy.node import Node
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from swarm_msgs.msg import Telemetry

from .submodules.transform_3d import Transform3D


class PeerLocalizer(Node):

    def __init__(self) -> None:
        super().__init__('peer_localizer')
        
        # params init
        
        parameters = [
            ('global_frame', ''),
            ('base_footprint', ''),
            ('lidar_frame', ''),
            ('telemetry_topic', ''),
            ('odom_topic', ''),
            ('scan_topic', ''),
            ('timer_period', 0.0),
            ('pacemaker_id', -1),
            ('fov_angles', ['', '']),
            ('fov_min_radius', -1.0),
            ('fov_max_radius', -1.0),
            ('robot_id', -1),
            ('gap', -1.0)
        ]

        self.declare_parameters(namespace='', parameters=parameters)
        
        self.global_frame = self.get_parameter('global_frame').value
        self.base_footprint = self.get_parameter('base_footprint').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        telemetry_topic = self.get_parameter('telemetry_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        period = self.get_parameter('timer_period').value
        self.pacemaker_id = self.get_parameter('pacemaker_id').value
        self.robot_id = self.get_parameter('robot_id').value
        self.gap = self.get_parameter('gap').value
        self.fov_angles = [eval(angle) for angle in self.get_parameter('fov_angles').value]
        self.fov_min_radius = self.get_parameter('fov_min_radius').value
        self.fov_max_radius = self.get_parameter('fov_max_radius').value

        self.get_logger().info(f'global_frame: {self.global_frame}')
        self.get_logger().info(f'base_footprint: {self.base_footprint}')
        self.get_logger().info(f'lidar_frame: {self.lidar_frame}')
        self.get_logger().info(f'telemetry_topic: {telemetry_topic}')
        self.get_logger().info(f'odom_topic: {odom_topic}')
        self.get_logger().info(f'scan_topic: {scan_topic}')
        self.get_logger().info(f'period: {period}')
        self.get_logger().info(f'pacemaker_id: {self.pacemaker_id}')
        self.get_logger().info(f'robot_id: {self.robot_id}')
        self.get_logger().info(f'gap: {self.gap}')
        self.get_logger().info(f'fov_angles: {self.fov_angles}')
        self.get_logger().info(f'fov_min_radius: {self.fov_min_radius}')
        self.get_logger().info(f'fov_max_radius: {self.fov_max_radius}')
        
        # arch init
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(period, self.callback)
        self.tf_timer = self.create_timer(0.01, self.tf_callback)
        
        self.create_subscription(Odometry, odom_topic, self.velocity_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.robot2_scan_callback, 10)

        # peer odom: robot we are following publishes its own odom at /robot{peer_id}/odom
        # we read peer.twist.linear.x as the leader's longitudinal speed for ACC MPC's v_rel
        peer_odom_topic = f'/robot{self.pacemaker_id}/odom'
        self.create_subscription(Odometry, peer_odom_topic, self.peer_odom_callback, 10)
        self.get_logger().info(f'peer_odom_topic: {peer_odom_topic}')

        self.telemetry_pub = self.create_publisher(Telemetry, telemetry_topic, 10)

        # vars init

        self.init_vels = False

        self.telemetry = None

        self.vels = (0.0, 0.0)

        # peer state from leader's odom
        self.peer_v_from_odom = 0.0
        self.peer_odom_stamp = None
        self.peer_odom_timeout = 0.5  # [s] consider peer odom stale beyond this

        self.lidar_to_bf = None
        self.bf_to_global = None
        
    def velocity_callback(self, msg : Odometry):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.vels = (v, w)

        self.init_vels = True

    def peer_odom_callback(self, msg: Odometry):
        # leader's longitudinal speed in its own frame (linear.x).
        # No projection through θ_peer−θ_follower yet — see future_work.md item 15.
        self.peer_v_from_odom = msg.twist.twist.linear.x
        self.peer_odom_stamp = self.get_clock().now()
        
    def robot2_scan_callback(self, msg : LaserScan):
        if self.lidar_to_bf is None or self.bf_to_global is None or not self.init_vels:
            return
                
        min_angle = msg.angle_min
        delta = msg.angle_increment
        
        tarr = []
        
        self.telemetry = Telemetry()
        
        for i, r in enumerate(msg.ranges):
            if r == float('inf') or r == float('nan') or not np.isfinite(r):
                continue
            
            angle = min_angle + i * delta
            
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            
            t_lidar = Transform3D.translation(x, y, 0.)
            
            t_bf = self.lidar_to_bf @ Transform3D.homogeneous(t_lidar)
            
            phi = np.arctan2(t_bf[1, 0], t_bf[0, 0])
            r = np.sqrt(t_bf[0, 0]**2 + t_bf[1, 0]**2)
            
            if r < self.fov_min_radius or self.fov_max_radius < r:
                continue
            
            if phi < self.fov_angles[0] or self.fov_angles[1] < phi:
                continue
            
            tarr.append(t_bf)
            
        if len(tarr) == 0:
            self.get_logger().info('NO POINTS')
            self.telemetry.is_valid = False
            return
            
        tarr = np.array(tarr)
        
        try:
            kmeans = KMeans(n_clusters=2, n_init='auto').fit(tarr[:, :3, :].squeeze())
            
            claster_center = kmeans.cluster_centers_[0] if kmeans.cluster_centers_[0, 0] < kmeans.cluster_centers_[1, 0] else kmeans.cluster_centers_[1]
            
            claster_center = claster_center.reshape(-1, 1)
            
            t = Transform3D.homogeneous(claster_center)
            
            assert t.shape == (4, 1)
            
        except:
            min_idx = np.where(np.min(tarr[:, 0, :]))
            t = tarr[min_idx].squeeze().reshape(-1, 1)
            
            assert t.shape == (4, 1)
        
        t_global = self.bf_to_global @ t
        
        self.valid_transforms = True
        
        tf = TransformStamped()
        
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.global_frame
        tf.child_frame_id = f'robot{self.robot_id}/peer'
        
        tf.transform.translation.x = t_global[0, 0]
        tf.transform.translation.y = t_global[1, 0]
        tf.transform.translation.z = 0.0
        
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        
        self.telemetry.x = self.bf_to_global.t[0, 0]
        self.telemetry.y = self.bf_to_global.t[1, 0]
        self.telemetry.theta = Transform3D.yaw(self.bf_to_global.R)
        self.telemetry.v, self.telemetry.w = self.vels
        self.telemetry.peer_x = t_global[0, 0]
        self.telemetry.peer_y = t_global[1, 0]

        # peer_v from leader's odometry; 0 if odom is missing/stale
        if self.peer_odom_stamp is not None:
            age = (self.get_clock().now() - self.peer_odom_stamp).nanoseconds * 1e-9
            self.telemetry.peer_v = self.peer_v_from_odom if age < self.peer_odom_timeout else 0.0
        else:
            self.telemetry.peer_v = 0.0

        self.telemetry.is_valid = True
        
    def callback(self):
        if self.telemetry is None:
            return
        self.telemetry_pub.publish(self.telemetry)
    
    def tf_callback(self):             
        # bf -> global        
        try:
            child = self.base_footprint
            parent = self.global_frame
            t = self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y
            translation = Transform3D.translation(x, y, 0.0)
            
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            theta = Rotation.from_quat(q).as_euler('xyz')[2]
            R = Transform3D.Rz(theta)
            
            self.bf_to_global = Transform3D(R, translation)
            
        except TransformException as ex:
            self.get_logger().info(f"Could not transform {parent} to {child}: {ex}")
        
        # lidar -> bf
        try:
            child = self.lidar_frame
            parent = self.base_footprint
            t = self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y
            translation = Transform3D.translation(x, y, 0.0)
            
            q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            theta = Rotation.from_quat(q).as_euler('xyz')[2]
            R = Transform3D.Rz(theta)
            
            self.lidar_to_bf = Transform3D(R, translation)
            
        except TransformException as ex:
            self.get_logger().info(f"Could not transform {parent} to {child}: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = PeerLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
