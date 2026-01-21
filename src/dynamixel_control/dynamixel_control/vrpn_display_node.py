# For terminal display pose:
# ros2 run dynamixel_control vrpn_display_node --ros-args -p topic_name:=/vrpn_mocap/ugv_1/pose

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
import math
import sys

class VrpnDisplayNode(Node):
    def __init__(self):
        super().__init__('vrpn_display_node')
        
        # Declare parameter for topic name
        self.declare_parameter('topic_name', '/vrpn_mocap/ugv_1/pose')
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            PoseStamped,
            self.topic_name,
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.get_logger().info(f"Subscribed to VRPN topic: {self.topic_name}")
        self.get_logger().info("Waiting for data... (Position will be logged every 0.5 seconds)")
        
        self.last_log_time = self.get_clock().now()
        self.log_period = 0.5 # Seconds

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        if (current_time - self.last_log_time).nanoseconds > self.log_period * 1e9:
            self.print_pose(msg)
            self.last_log_time = current_time

    def print_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        
        roll, pitch, yaw = self.get_euler_from_quaternion(q)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        log_msg = (
            f"\n--- VRPN Data [{self.topic_name}] ---\n"
            f"Position (m):    x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}\n"
            f"Euler (deg):     R={roll_deg:.2f}, P={pitch_deg:.2f}, Y={yaw_deg:.2f}\n"
            f"Quaternion:      x={q.x:.3f}, y={q.y:.3f}, z={q.z:.3f}, w={q.w:.3f}\n"
            f"----------------------------------------"
        )
        self.get_logger().info(log_msg)

    def get_euler_from_quaternion(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        
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
        
        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)
    node = VrpnDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
