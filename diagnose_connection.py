import serial
import time
import glob
import sys

def check_port(port, baudrate=921600, timeout=1):
    try:
        # Check if port is accessible
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        print(f"[-] Checking {port} ... Open success.")
        
        # Try to read some bytes
        data = ser.read(100)
        ser.close()
        
        if len(data) > 0:
            print(f"[!] DATA DETECTED on {port}! (Read {len(data)} bytes)")
            print(f"    Sample hex: {data[:20].hex()}")
            return True
        else:
            print(f"[-] {port} is open but NO DATA received (Silence).")
            return False
            
    except serial.SerialException as e:
        if "Device or resource busy" in str(e):
            print(f"[!] {port} is BUSY (Likely being used by MicroXRCEAgent).")
            # If it's busy, it might be the right one, or user just forgot to close it.
            return "BUSY"
        else:
            print(f"[x] Could not open {port}: {e}")
            return False

def main():
    ports = glob.glob('/dev/ttyUSB*')
    baudrates = [921600, 57600, 115200, 460800, 3000000]
    
    if not ports:
        print("No /dev/ttyUSB* ports found!")
        return

    print(f"Found ports: {ports}")
    print(f"Scanning baudrates: {baudrates}")
    print("-" * 40)
    
    candidates = []
    
    for port in sorted(ports):
        for baud in baudrates:
            print(f"Checking {port} @ {baud}...")
            result = check_port(port, baudrate=baud, timeout=0.5)
            if result is True:
                candidates.append((port, baud))
                break # Found a valid baudrate for this port
            elif result == "BUSY":
                print(f"  -> Port {port} is BUSY (skip)")
                break 
            
    print("-" * 40)
    print("DIAGNOSTIC RESULT:")
    
    if candidates:
        for port, baud in candidates:
            print(f"SUCCESS: Data stream detected on: {port} @ {baud} baud")
            print(f"ACTION: Please try running MicroXRCEAgent on {port} with -b {baud}")
            print(f"        Command: MicroXRCEAgent serial -D {port} -b {baud}")
    else:
        print("FAILURE: No data detected on any port with any common baudrate.")
        print("Possible causes:")
        print("1. Port is BUSY (Agent is running? Stop it first).")
        print("2. RX/TX wires are swapped (Hardware issue).")
        print("3. Flight controller not sending data (Check 'uxrce_dds_client status' in MAVLink Console).")

if __name__ == "__main__":
    main()
