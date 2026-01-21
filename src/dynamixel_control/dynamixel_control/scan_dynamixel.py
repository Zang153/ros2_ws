import sys
import os
from dynamixel_sdk import *

def get_available_port():
    """
    Scans for Dynamixel motors on common ports and returns the first port where motors are found.
    Returns None if no motors are found.
    """
    # Device names to scan
    DEVICENAMES = ['/dev/ttyUSB0', '/dev/ttyUSB1']

    # Baudrates to scan
    BAUDRATES = [57600, 1000000, 9600, 115200, 3000000, 4000000]
    
    # Protocol version
    PROTOCOL_VERSION = 2.0

    print("Scanning for Dynamixel ports...")

    for device_name in DEVICENAMES:
        # Initialize PortHandler and PacketHandler
        portHandler = PortHandler(device_name)
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not portHandler.openPort():
            continue

        for baud in BAUDRATES:
            if not portHandler.setBaudRate(baud):
                continue

            # Check for at least one motor (ID 1-20)
            for dxl_id in range(1, 21):
                dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
                if dxl_comm_result == COMM_SUCCESS:
                    print(f"Found Dynamixel on {device_name} at {baud} baud")
                    portHandler.closePort()
                    return device_name, baud

        portHandler.closePort()
    
    print("No Dynamixel motors found on standard ports.")
    return None, None

def main():
    print("Dynamixel Scanner")
    print("-----------------")

    # Protocol version
    PROTOCOL_VERSION = 2.0

    # Device names to scan
    DEVICENAMES = ['/dev/ttyUSB0', '/dev/ttyUSB1']

    # Baudrates to scan
    BAUDRATES = [57600, 1000000, 9600, 115200, 3000000, 4000000]

    found_motors = []

    for device_name in DEVICENAMES:
        print(f"\nScanning port: {device_name}")
        
        # Initialize PortHandler and PacketHandler
        portHandler = PortHandler(device_name)
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not portHandler.openPort():
            print(f"Failed to open port {device_name}")
            print("Check connection and permissions.")
            continue

        for baud in BAUDRATES:
            print(f"Scanning at {baud} baud on {device_name}...")
            if not portHandler.setBaudRate(baud):
                print(f"Failed to set baudrate to {baud}")
                continue

            # Scan IDs 1 to 20 (assuming user has IDs in this range)
            # You can increase range if needed
            for dxl_id in range(1, 21):
                dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
                if dxl_comm_result == COMM_SUCCESS:
                    print(f"  [SUCCESS] Found ID {dxl_id} at {baud} baud! (Model: {dxl_model_number}) on {device_name}")
                    found_motors.append({'id': dxl_id, 'baud': baud, 'model': dxl_model_number, 'port': device_name})
                else:
                    # print(f"    Failed ID {dxl_id}")
                    pass

        portHandler.closePort()

    print("\nScan Complete.")
    if found_motors:
        print("Found Motors:")
        for motor in found_motors:
            print(f"  ID: {motor['id']}, Baud: {motor['baud']}, Model: {motor['model']}, Port: {motor['port']}")
    else:
        print("No motors found. Check power, cables, and Protocol version.")

if __name__ == '__main__':
    main()
