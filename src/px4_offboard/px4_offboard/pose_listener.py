#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
import math
import sys

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        
        # Match the QoS settings of the publisher (often VRPN client or bridge)
        # Using Best Effort and Volatile as seen in mocap_bridge.py
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        topic_name = '/vrpn_mocap/AM/pose'
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.listener_callback,
            qos_profile)
        
        self.get_logger().info(f'Subscribed to {topic_name}')
        print(f"Waiting for data on {topic_name}...")

    def listener_callback(self, msg):
        # Extract position
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z
        
        # Extract orientation
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w
        
        # Convert to Euler angles
        roll, pitch, yaw = self.euler_from_quaternion(q_x, q_y, q_z, q_w)
        
        # Convert to degrees for display
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Print to console
        # Using carriage return to update the same lines if possible, or just print log
        # For simplicity and scrolling log:
        output = (
            f"Position [m]: x={pos_x:.4f}, y={pos_y:.4f}, z={pos_z:.4f} | "
            f"Attitude [deg]: Roll={roll_deg:.2f}, Pitch={pitch_deg:.2f}, Yaw={yaw_deg:.2f}"
        )
        print(output)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
