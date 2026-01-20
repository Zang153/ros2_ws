import sys
import os
from dynamixel_sdk import *

def main():
    print("Dynamixel Scanner")
    print("-----------------")

    # Protocol version
    PROTOCOL_VERSION = 2.0

    # Device name
    DEVICENAME = '/dev/ttyUSB0'

    # Baudrates to scan
    BAUDRATES = [57600, 1000000, 9600, 115200, 3000000, 4000000]

    # Initialize PortHandler and PacketHandler
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    found_motors = []

    if not portHandler.openPort():
        print(f"Failed to open port {DEVICENAME}")
        print("Check connection and permissions.")
        return

    for baud in BAUDRATES:
        print(f"Scanning at {baud} baud...")
        if not portHandler.setBaudRate(baud):
            print(f"Failed to set baudrate to {baud}")
            continue

        # Scan IDs 1 to 20 (assuming user has IDs in this range)
        # You can increase range if needed
        for dxl_id in range(1, 21):
            dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"  [SUCCESS] Found ID {dxl_id} at {baud} baud! (Model: {dxl_model_number})")
                found_motors.append({'id': dxl_id, 'baud': baud, 'model': dxl_model_number})
            else:
                # print(f"    Failed ID {dxl_id}")
                pass

    portHandler.closePort()

    print("\nScan Complete.")
    if found_motors:
        print("Found Motors:")
        for motor in found_motors:
            print(f"  ID: {motor['id']}, Baud: {motor['baud']}, Model: {motor['model']}")
    else:
        print("No motors found. Check power, cables, and Protocol version.")

if __name__ == '__main__':
    main()
