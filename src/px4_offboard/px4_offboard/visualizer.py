#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleOdometry
import sys
import os

class Visualizer(Node):

    def __init__(self):
        super().__init__('visualizer')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.vehicle_odometry_callback,
            qos_profile)

        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_odometry = VehicleOdometry()

        self.create_timer(0.2, self.timer_callback)  # 5 Hz refresh

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def timer_callback(self):
        os.system('clear')
        print("="*50)
        print("PX4 OFFBOARD CONTROL STATUS DASHBOARD")
        print("="*50)
        
        # System Status
        nav_state = self.vehicle_status.nav_state
        arming_state = self.vehicle_status.arming_state
        
        nav_state_str = "OFFBOARD" if nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD else f"OTHER ({nav_state})"
        arming_state_str = "ARMED" if arming_state == VehicleStatus.ARMING_STATE_ARMED else "DISARMED"
        
        print(f"Nav State:    {nav_state_str}")
        print(f"Arming State: {arming_state_str}")
        print("-" * 50)
        
        # Position (Local Frame)
        pos = self.vehicle_local_position
        print(f"Local Position (NED):")
        print(f"  X: {pos.x:.3f} m")
        print(f"  Y: {pos.y:.3f} m")
        print(f"  Z: {pos.z:.3f} m (Negative is UP)")
        print("-" * 50)
        
        # Mocap Input
        odom = self.vehicle_odometry
        print(f"Mocap Input (NED):")
        if hasattr(odom, 'position'):
             # Handle array vs distinct fields depending on message definition version, usually array for position in VehicleOdometry
             if len(odom.position) >= 3:
                print(f"  X: {odom.position[0]:.3f} m")
                print(f"  Y: {odom.position[1]:.3f} m")
                print(f"  Z: {odom.position[2]:.3f} m")
        print("="*50)
        print("Press Ctrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
