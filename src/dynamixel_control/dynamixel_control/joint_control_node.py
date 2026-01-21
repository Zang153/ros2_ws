import sys
import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
import threading
from sensor_msgs.msg import JointState

# Check if dynamixel_sdk is available
try:
    from dynamixel_sdk import *
except ImportError:
    print("Failed to import dynamixel_sdk. Please ensure it is installed (pip install dynamixel-sdk).")
    sys.exit(1)

# Control Table Addresses for X Series (e.g., XL330, XL430, XC430, etc.)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

from dynamixel_control.scan_dynamixel import get_available_port

# Protocol version
PROTOCOL_VERSION            = 2.0

# Default setting (Will be overwritten by scan)
BAUDRATE                    = 57600
DEVICENAME                  = '/dev/ttyUSB1'

# Try to detect port automatically
detected_port, detected_baud = get_available_port()
if detected_port:
    print(f"Automatically detected Dynamixel on {detected_port} at {detected_baud} baud.")
    DEVICENAME = detected_port
    BAUDRATE = detected_baud
else:
    print(f"Could not detect Dynamixel. Using default: {DEVICENAME} at {BAUDRATE} baud.")

TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                 

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_node')
        self.get_logger().info("Initializing Joint Control Node...")

        # Initialize Dynamixel SDK handlers
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Thread Lock for Dynamixel Access
        self.dxl_lock = threading.Lock()
        
        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, 4)
        
        # Initialize GroupSyncRead instance for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, 4)

        # Open port
        if self.portHandler.openPort():
            self.get_logger().info(f"Succeeded to open the port {DEVICENAME}")
        else:
            self.get_logger().error(f"Failed to open the port {DEVICENAME}")
            self.get_logger().error("Please check the USB connection and permissions (e.g., sudo chmod 666 /dev/ttyUSB0)")
            # sys.exit(1)

        # Set baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info(f"Succeeded to change the baudrate to {BAUDRATE}")
        else:
            self.get_logger().error(f"Failed to change the baudrate to {BAUDRATE}")
            
        # Motor IDs to control
        self.motor_ids = [1, 2, 3]
        
        # Add parameters for GroupSyncRead
        for dxl_id in self.motor_ids:
            if not self.groupSyncRead.addParam(dxl_id):
                self.get_logger().error(f"Failed to add GroupSyncRead param for ID {dxl_id}")
        
        # Enable Torque
        self.enable_torque()

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_target_pub = self.create_publisher(JointState, 'joint_targets', 10)

        # Current Target Angles (Logic Frame)
        self.target_angles = [0.0, 0.0, 0.0]
        
        # Timer for Reading/Publishing (100Hz)
        self.timer = self.create_timer(0.01, self.update_loop)
        
        # Start Input Thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
    def enable_torque(self):
        with self.dxl_lock:
             for dxl_id in self.motor_ids:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(f"ID {dxl_id} Torque Enable Failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    self.get_logger().warn(f"ID {dxl_id} Torque Enable Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                else:
                    self.get_logger().info(f"Dynamixel #{dxl_id} has been successfully connected and torque enabled")

    def input_loop(self):
        print("\n=== Manual Joint Control ===")
        print("Enter 3 angles (degrees) separated by space.")
        print("Example: 0 0 0")
        print("Or 'q' to quit.")
        
        while rclpy.ok():
            try:
                # Use a non-blocking input method or just handle shutdown cleanly
                # Python's input() is blocking. 
                # To quit cleanly on 'q', we just break loop.
                # To handle Ctrl+C, we rely on main thread raising KeyboardInterrupt?
                # Actually, input() will raise EOFError on Ctrl+D, but Ctrl+C might be caught by main thread.
                
                user_input = input("Enter angles (deg) > ")
                if user_input.lower() == 'q':
                    print("Quitting...")
                    # Do not call rclpy.shutdown() here, let main handle it
                    # Just break the loop
                    break
                
                parts = user_input.split()
                if len(parts) != 3:
                    print("Error: Please enter exactly 3 numbers.")
                    continue
                    
                angles = [float(p) for p in parts]
                
                # Update targets
                self.target_angles = angles
                print(f"Moving to: {self.target_angles}")
                
                # Send command immediately
                with self.dxl_lock:
                    self.send_motor_command(self.target_angles)
                
            except ValueError:
                print("Error: Invalid number format.")
            except Exception as e:
                print(f"Error: {e}")

    def send_motor_command(self, angles):
        # Convert Logic Angles -> Dynamixel Angles
        # Mapping: Dynamixel = 180.0 - Logic
        
        dyn_angles = 180.0 - np.array(angles)
        
        # We assume lock is acquired by caller (input_loop calls this inside lock)
        # But to be safe, we can check or re-acquire if RLock? 
        # Since we use simple Lock, we must ensure it's not re-entered if we lock here.
        # input_loop calls this inside `with self.dxl_lock:`. 
        # So we should NOT lock here again unless we change input_loop.
        # Let's keep it lock-free and rely on caller, OR move lock inside here.
        # I moved lock to call site in input_loop.
        
        self.groupSyncWrite.clearParam()
        
        for i, dxl_id in enumerate(self.motor_ids):
            angle = dyn_angles[i]
            
            # Safety Clamp (150 to 270)
            if angle < 150.0:
                angle = 150.0
            elif angle > 270.0:
                angle = 270.0
                
            # Convert to Value (0-4095)
            position_value = int(angle * 4095.0 / 360.0)
            
            # Allocate 4 bytes
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(position_value)),
                DXL_HIBYTE(DXL_LOWORD(position_value)),
                DXL_LOBYTE(DXL_HIWORD(position_value)),
                DXL_HIBYTE(DXL_HIWORD(position_value))
            ]
            
            if not self.groupSyncWrite.addParam(dxl_id, param_goal_position):
                self.get_logger().error(f"Failed to add param for ID {dxl_id}")

        # Send SyncWrite Packet
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn(f"SyncWrite Failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        # Publish Target for Visualizer
        target_msg = JointState()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.name = [f'motor_{id}' for id in self.motor_ids]
        target_msg.position = [float(a) for a in angles]
        self.joint_target_pub.publish(target_msg)

    def update_loop(self):
        with self.dxl_lock:
            # 1. Read Current Positions
            dxl_comm_result = self.groupSyncRead.txRxPacket()
        
            read_angles = []
            if dxl_comm_result == COMM_SUCCESS:
                for i, dxl_id in enumerate(self.motor_ids):
                    if self.groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 4):
                        present_position = self.groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, 4)
                        
                        # Convert Value (0-4095) -> Dynamixel Angle (0-360)
                        present_angle_dyn = present_position * 360.0 / 4095.0
                        
                        # Convert Dynamixel Angle -> Logic Angle
                        present_angle_logic = 180.0 - present_angle_dyn
                        read_angles.append(present_angle_logic)
                    else:
                        read_angles.append(None)
            else:
                 # Only warn occasionally or if persistent
                 # self.get_logger().warn(f"SyncRead Failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                 read_angles = []

        # 2. Publish Joint States
        if read_angles and None not in read_angles:
             joint_msg = JointState()
             joint_msg.header.stamp = self.get_clock().now().to_msg()
             joint_msg.name = [f'motor_{id}' for id in self.motor_ids]
             joint_msg.position = [float(a) for a in read_angles]
             self.joint_pub.publish(joint_msg)
             
        # We don't necessarily need to resend the command every loop if using Position Control,
        # but Dynamixel sometimes times out if we don't refresh? 
        # Actually, for Position Control, writing once is enough.
        # But to keep it stiff/active, we can just let the motor handle it.
        # Re-writing the same value is fine too.

def main(args=None):
    rclpy.init(args=args)
    node = JointControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Ensure input thread dies? It's daemon, so it should die with main thread.

if __name__ == '__main__':
    main()
