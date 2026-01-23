import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus

import time

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # QoS Profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_subscriber_ = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Variables
        self.vehicle_status = None
        self.offboard_setpoint_counter_ = 0
        self.takeoff_height = -1.5 # NED (Negative is Up)
        self.is_armed = False
        self.is_offboard = False

        # Timer
        self.timer_period = 0.1  # 100 milliseconds (10Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("Offboard Control Node Started")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        # Check arming state (arming_state == 2 is armed)
        self.is_armed = (msg.arming_state == 2)
        # Check navigation state (nav_state == 14 is offboard)
        self.is_offboard = (msg.nav_state == 14)

    def timer_callback(self):
        if self.offboard_setpoint_counter_ == 100:
            self.engage_offboard_mode()
            self.arm()

        # Publish heartbeat and setpoints
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter_ < 101:
            self.offboard_setpoint_counter_ += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # NED Coordinates: [North, East, Down]
        # Fixed point takeoff: 0, 0, -1.5m
        msg.position = [0.0, 0.0, self.takeoff_height]
        msg.yaw = 0.0 # Facing North

        self.trajectory_setpoint_publisher_.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, # Custom mode
            param2=6.0  # OFFBOARD mode
        )
        self.get_logger().info("Switching to Offboard Mode")

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0  # Arm
        )
        self.get_logger().info("Arming command sent")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
