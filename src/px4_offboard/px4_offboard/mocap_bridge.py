#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry

import math

class MocapBridge(Node):

    def __init__(self):
        super().__init__('mocap_bridge')

        # Parameters
        self.declare_parameter('pose_topic', '/vrpn_mocap/AM/pose')
        self.declare_parameter('px4_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('frequency', 0)  # Default 30 Hz. Set to 0 for unlimited (input rate)
        
        self.pose_topic = self.get_parameter('pose_topic').value
        self.px4_topic = self.get_parameter('px4_topic').value
        self.frequency = self.get_parameter('frequency').value

        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # QoS for Mocap (Best Effort to match publisher)
        mocap_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            mocap_qos_profile
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            VehicleOdometry,
            self.px4_topic,
            qos_profile
        )
        
        self.latest_pose_msg = None
        
        if self.frequency > 0:
            timer_period = 1.0 / self.frequency
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(f"Mocap Bridge Started. Listening on {self.pose_topic}, publishing to {self.px4_topic} at {self.frequency} Hz")
        else:
            self.get_logger().info(f"Mocap Bridge Started. Listening on {self.pose_topic}, publishing to {self.px4_topic} at input rate")

    def pose_callback(self, msg):
        if self.frequency > 0:
            self.latest_pose_msg = msg
        else:
            self.publish_odometry(msg)

    def timer_callback(self):
        if self.latest_pose_msg is not None:
            self.publish_odometry(self.latest_pose_msg)

    def publish_odometry(self, msg):
        odom_msg = VehicleOdometry()

        # Timestamp sync (convert nanoseconds to microseconds)
        # PX4 expects timestamps in microseconds
        odom_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        odom_msg.timestamp_sample = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)

        # Frame ID
        # POSE_FRAME_NED = 0 (Check definition, but 0 is usually unknown/NED in older versions)
        # PX4 v1.14+ usually infers from topic or uses explicit frame.
        # We will assume NED data.
        odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # Position Conversion (ENU -> NED)
        # X_ned = Y_enu
        # Y_ned = X_enu
        # Z_ned = -Z_enu
        odom_msg.position = [msg.pose.position.y, msg.pose.position.x, -msg.pose.position.z]

        # Orientation Conversion
        # ROS: x, y, z, w
        # PX4: w, x, y, z
        
        # Input Quaternion (Body -> ENU)
        q_enu = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        
        # Rotation Quaternion (ENU -> NED)
        # 180 degree rotation around vector (1, 1, 0)
        # q_rot = [0, 1/sqrt(2), 1/sqrt(2), 0]
        c = 0.70710678
        q_rot = [0.0, c, c, 0.0]

        # Output Quaternion (Body -> NED) = q_rot * q_enu
        # q = q_a * q_b
        # w = w_a w_b - x_a x_b - y_a y_b - z_a z_b
        # x = w_a x_b + x_a w_b + y_a z_b - z_a y_b
        # y = w_a y_b - x_a z_b + y_a w_b + z_a x_b
        # z = w_a z_b + x_a y_b - y_a x_b + z_a w_b
        
        w_a, x_a, y_a, z_a = q_rot
        w_b, x_b, y_b, z_b = q_enu

        w = w_a*w_b - x_a*x_b - y_a*y_b - z_a*z_b
        x = w_a*x_b + x_a*w_b + y_a*z_b - z_a*y_b
        y = w_a*y_b - x_a*z_b + y_a*w_b + z_a*x_b
        z = w_a*z_b + x_a*y_b - y_a*x_b + z_a*w_b

        odom_msg.q = [w, x, y, z]

        # Velocity - Set to NaN as we don't have it from PoseStamped
        odom_msg.velocity = [float('nan'), float('nan'), float('nan')]
        odom_msg.angular_velocity = [float('nan'), float('nan'), float('nan')]

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
