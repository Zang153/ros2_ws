import sys
import os
import rclpy
from rclpy.node import Node
import numpy as np
import time

# Check if dynamixel_sdk is available
try:
    from dynamixel_sdk import *
except ImportError:
    print("Failed to import dynamixel_sdk. Please ensure it is installed (pip install dynamixel-sdk).")
    sys.exit(1)

from dynamixel_control.delta_kinematics import DeltaKinematics

# Control Table Addresses for X Series (e.g., XL330, XL430, XC430, etc.)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

# Protocol version
PROTOCOL_VERSION            = 2.0

# Default setting
BAUDRATE                    = 57600             
DEVICENAME                  = '/dev/ttyUSB0'    

TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                 

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info("Initializing Motor Controller...")

        # Initialize Delta Kinematics
        self.kinematics = DeltaKinematics()

        # Initialize Dynamixel SDK handlers
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
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
        
        # Enable Torque and Check Connection
        self.enable_torque()

        # Trajectory Parameters
        self.traj_radius = 0.12
        self.traj_z = -0.20
        self.traj_period = 3.0 # seconds
        self.start_time = time.time()

        # Create Timer for Control Loop (100Hz)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Control Loop Started at 100Hz")
        
        # Counter for printing
        self.loop_counter = 0

    def enable_torque(self):
        for dxl_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f"ID {dxl_id} Torque Enable Failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().warn(f"ID {dxl_id} Torque Enable Error: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Dynamixel#{dxl_id} has been successfully connected and torque enabled")

    def get_circular_trajectory(self, t):
        omega = 2 * np.pi / self.traj_period
        phase = omega * t
        
        x = self.traj_radius * np.cos(phase)
        y = self.traj_radius * np.sin(phase)
        z = self.traj_z
        
        return np.array([x, y, z])

    def control_loop(self):
        t = time.time() - self.start_time
        self.loop_counter += 1
        
        # --- 1. Generate Trajectory Point ---
        des_pos = self.get_circular_trajectory(t)
        
        # --- 2. Inverse Kinematics ---
        # Returns angles in degrees (MuJoCo frame: 30 to -90 range approx)
        mujoco_angles = self.kinematics.ik(des_pos)
        
        if isinstance(mujoco_angles, int) and mujoco_angles == -1:
            self.get_logger().warn(f"IK Failed for pos: {des_pos}")
            return

        # --- 3. Map to Dynamixel Angles ---
        # Mapping: Dynamixel = MuJoCo + 240
        # Range check: 150 to 270
        dyn_angles = mujoco_angles + 240.0
        
        # --- 4. Prepare SyncWrite ---
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

        # --- 5. Send SyncWrite Packet ---
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            pass # Timeout or error usually
            
        # --- 6. Read Current Positions (SyncRead) ---
        # Perform read every loop or every N loops? 
        # Reading takes time, might slow down loop. But user wants visualization.
        # Let's try every loop, but print every 10th.
        
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        
        if dxl_comm_result == COMM_SUCCESS:
            # Check if data is available for all motors
            read_angles = []
            for i, dxl_id in enumerate(self.motor_ids):
                if self.groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 4):
                    present_position = self.groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, 4)
                    
                    # Convert Value (0-4095) -> Dynamixel Angle (0-360)
                    present_angle_dyn = present_position * 360.0 / 4095.0
                    
                    # Convert Dynamixel Angle -> MuJoCo Angle
                    # Formula: MuJoCo = Dyn - 240
                    present_angle_mujoco = present_angle_dyn - 240.0
                    read_angles.append(present_angle_mujoco)
                else:
                    read_angles.append(None)
            
            # Print every 20 loops (approx 5Hz)
            if self.loop_counter % 20 == 0:
                # Format output
                status_str = f"[Time {t:.2f}s] Pos: "
                for i, ang in enumerate(read_angles):
                    if ang is not None:
                        status_str += f"ID{self.motor_ids[i]}: {ang:.1f}°  "
                    else:
                        status_str += f"ID{self.motor_ids[i]}: ???  "
                print(status_str)
        else:
             # If read fails, maybe print error occasionally
             if self.loop_counter % 100 == 0:
                 # self.get_logger().warn(f"SyncRead Failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                 pass

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Disable Torque
        for dxl_id in motor_controller.motor_ids:
            motor_controller.packetHandler.write1ByteTxRx(
                motor_controller.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
        motor_controller.portHandler.closePort()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
