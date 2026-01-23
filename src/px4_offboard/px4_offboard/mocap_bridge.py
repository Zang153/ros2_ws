import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry

import math
import numpy as np

class MocapBridge(Node):
    def __init__(self):
        super().__init__('mocap_bridge')

        # Parameters
        self.declare_parameter('pose_topic', '/vrpn_client/rigid_body/pose')
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        # QoS for PX4 (Best Effort is often required for high-rate sensor data)
        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for VRPN (Reliable is default for mocap, but check if needed)
        qos_profile_vrpn = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile_vrpn)

        # Publisher
        self.vehicle_odometry_publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_odometry',
            qos_profile_px4)

        self.get_logger().info(f"Mocap Bridge Started. Subscribing to {self.pose_topic}")

    def pose_callback(self, msg):
        vehicle_odometry = VehicleOdometry()

        # Timestamp: Convert ROS time (nanoseconds) to PX4 time (microseconds)
        # Note: PX4 expects synchronized time. If using XRCE-DDS with timesync, 
        # using the current agent time might be better or just passing through if synced.
        # Here we use the timestamp from the message to minimize latency jitter.
        now = self.get_clock().now()
        vehicle_odometry.timestamp = int(now.nanoseconds / 1000)
        vehicle_odometry.timestamp_sample = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)

        # Frame Conversion: ENU (ROS) -> NED (PX4)
        # Position
        # PX4 X (North) = ROS Y (North) (if aligned) or ROS X (East)?
        # Standard ENU: X=East, Y=North, Z=Up
        # Standard NED: X=North, Y=East, Z=Down
        # So: Px = Ry, Py = Rx, Pz = -Rz
        
        vehicle_odometry.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vehicle_odometry.position = [msg.pose.position.y, msg.pose.position.x, -msg.pose.position.z]

        # Orientation (Quaternion)
        # ROS: (x, y, z, w)
        # PX4: (w, x, y, z) - Wait, px4_msgs use standard [w, x, y, z] or [x, y, z, w]?
        # Checking px4_msgs definitions: float32[4] q. usually it is [w, x, y, z] in PX4 internal, 
        # but in ROS 2 messages it depends on the mapping. 
        # Actually px4_msgs/VehicleOdometry.msg defines `float32[4] q`
        # Standard PX4 convention is [w, x, y, z].
        # ROS convention is [x, y, z, w].
        # We need to construct the NED quaternion.
        
        q_enu = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_ned = self.enu_to_ned_orientation(q_enu)
        
        vehicle_odometry.q = q_ned

        # Velocity - Optional, set to NaN if unknown
        vehicle_odometry.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        vehicle_odometry.velocity = [float('nan'), float('nan'), float('nan')]
        vehicle_odometry.angular_velocity = [float('nan'), float('nan'), float('nan')]

        self.vehicle_odometry_publisher.publish(vehicle_odometry)

    def enu_to_ned_orientation(self, q_enu):
        # q_enu: [x, y, z, w]
        # Return: [w, x, y, z] (PX4 convention)
        
        # Rotation from ENU to NED is:
        # Rotate 90 deg around Z (East becomes North? No. X_enu=East, Y_ned=East. )
        # Let's map axes:
        # X_ned (North) = Y_enu
        # Y_ned (East) = X_enu
        # Z_ned (Down) = -Z_enu
        
        # This corresponds to a rotation of:
        # 1. Swap X/Y (Reflection? No, Rotation -90Z aligns X->-Y? No)
        # Let's use the rotation matrix approach to be safe.
        
        # Quaternion to Matrix
        x, y, z, w = q_enu
        
        # Create rotation matrix for q_enu
        # (Standard formula)
        R_enu = self.quat_to_mat(x, y, z, w)

        # Transform Matrix
        # M = [ [0, 1, 0], [1, 0, 0], [0, 0, -1] ]
        M = np.array([[0, 1, 0],
                      [1, 0, 0],
                      [0, 0, -1]])
        
        # R_ned = M * R_enu * M.T  (Coordinate System Transform)
        # OR is it R_ned = M * R_enu? 
        # If R_enu takes vector in Body to ENU: v_enu = R_enu * v_body
        # We want v_ned = R_ned * v_body
        # And we know v_ned = M * v_enu
        # So v_ned = M * (R_enu * v_body) -> R_ned = M * R_enu
        
        R_ned = M @ R_enu

        # Matrix to Quaternion
        q_ned = self.mat_to_quat(R_ned)
        
        return q_ned

    def quat_to_mat(self, x, y, z, w):
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])

    def mat_to_quat(self, R):
        trace = np.trace(R)
        if trace > 0:
            S = math.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif (R[1,1] > R[2,2]):
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        
        # Return as [w, x, y, z]
        return [w, x, y, z]

def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
