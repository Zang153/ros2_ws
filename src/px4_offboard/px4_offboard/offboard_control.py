#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

import time

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # Parameters
        self.declare_parameter('takeoff_height', -1.0)
        self.declare_parameter('hover_duration', 10.0)
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.hover_duration = self.get_parameter('hover_duration').value

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        
        # State Machine
        self.STATE_INIT = 0
        self.STATE_TAKEOFF = 1
        self.STATE_HOVER = 2
        self.STATE_LAND = 3
        self.STATE_DISARM = 4
        self.STATE_END = 5
        
        self.current_state = self.STATE_INIT
        self.state_start_time = 0.0

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0 # NED frame, 0 is North
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()

        if self.current_state == self.STATE_INIT:
            # Send setpoints before offboard
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            if self.offboard_setpoint_counter >= 10:
                self.engage_offboard_mode()
                self.arm()
                self.current_state = self.STATE_TAKEOFF
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info("Taking off...")
            self.offboard_setpoint_counter += 1

        elif self.current_state == self.STATE_TAKEOFF:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            # Check if reached altitude (within 0.2m)
            # NED Z is negative up. takeoff_height is e.g. -1.0. 
            # If current z (-1.0) < target (-0.8)? No.
            # If target is -1.0, and we are at 0.0 (ground).
            # We want z <= -0.8 (closer to -1.0 than -0.2)
            if self.vehicle_local_position.z <= (self.takeoff_height + 0.2):
                self.current_state = self.STATE_HOVER
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info("Target altitude reached. Hovering...")

        elif self.current_state == self.STATE_HOVER:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.state_start_time) > self.hover_duration:
                self.current_state = self.STATE_LAND
                self.land()
                self.get_logger().info("Hover time expired. Landing...")

        elif self.current_state == self.STATE_LAND:
            # In Land mode, the vehicle manages itself, but we should check if disarmed
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.current_state = self.STATE_END
                self.get_logger().info("Vehicle disarmed. Mission complete.")
            
        elif self.current_state == self.STATE_END:
            # Nothing to do
            pass

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
