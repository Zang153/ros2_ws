import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import threading
import numpy as np
from collections import deque
from dynamixel_control.delta_kinematics import DeltaKinematics
import curses
import time

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        self.kinematics = DeltaKinematics()
        
        # Current Data Storage
        self.motor_target = [0.0, 0.0, 0.0]
        self.motor_current = [0.0, 0.0, 0.0]
        
        self.ee_target = [0.0, 0.0, 0.0]
        self.ee_current = [0.0, 0.0, 0.0] # FK calculated
        
        self.ik_error_count = 0
        self.last_update_time = 0.0
        
        # Subscribers
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.create_subscription(JointState, 'joint_targets', self.target_callback, 10)
        self.create_subscription(Point, 'delta/target_pos', self.ee_target_callback, 10)
        
    def joint_callback(self, msg):
        # Assume msg.position is [m1, m2, m3]
        pos = msg.position
        for i in range(3):
            if i < len(pos):
                self.motor_current[i] = pos[i]
        
        # Calculate FK
        try:
            fk_pos = self.kinematics.fk(pos)
            if not isinstance(fk_pos, int): # fk returns -1 on failure
                self.ee_current = fk_pos
            else:
                 self.ik_error_count += 1
        except Exception as e:
            self.get_logger().error(f"FK Error: {e}")

    def target_callback(self, msg):
        pos = msg.position
        for i in range(3):
            if i < len(pos):
                self.motor_target[i] = pos[i]

    def ee_target_callback(self, msg):
        self.ee_target = [msg.x, msg.y, msg.z]

def draw_loop(stdscr, node):
    # Setup curses
    curses.curs_set(0) # Hide cursor
    stdscr.nodelay(True) # Non-blocking input
    stdscr.timeout(100) # Refresh every 100ms
    
    # Colors
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)
    
    header_attr = curses.color_pair(2) | curses.A_BOLD
    label_attr = curses.color_pair(3)
    val_attr = curses.color_pair(1)
    
    while rclpy.ok():
        stdscr.erase()
        
        # Title
        stdscr.addstr(1, 2, "Delta Robot Control Monitor", header_attr | curses.A_UNDERLINE)
        stdscr.addstr(2, 2, "Press 'q' to quit", curses.A_DIM)

        # Motor Status Table
        row = 4
        stdscr.addstr(row, 2, "Motor Status (Degrees):", header_attr)
        row += 2
        stdscr.addstr(row, 4, f"{'ID':<5} {'Target':<15} {'Current':<15} {'Error':<15}", label_attr)
        row += 1
        stdscr.addstr(row, 4, "-" * 50)
        row += 1
        
        for i in range(3):
            target = node.motor_target[i]
            current = node.motor_current[i]
            error = target - current
            
            stdscr.addstr(row, 4, f"{i+1:<5} {target:>10.2f}     {current:>10.2f}     {error:>10.2f}", val_attr)
            row += 1
            
        # End-Effector Status
        row += 2
        stdscr.addstr(row, 2, "End-Effector Position (Meters):", header_attr)
        row += 2
        stdscr.addstr(row, 4, f"{'Axis':<5} {'Target':<15} {'FK Calculated':<15} {'Error':<15}", label_attr)
        row += 1
        stdscr.addstr(row, 4, "-" * 50)
        row += 1
        
        axes = ['X', 'Y', 'Z']
        for i in range(3):
            target = node.ee_target[i]
            current = node.ee_current[i]
            error = target - current
            
            stdscr.addstr(row, 4, f"{axes[i]:<5} {target:>10.4f}     {current:>10.4f}     {error:>10.4f}", val_attr)
            row += 1
            
        # Stats
        row += 2
        stdscr.addstr(row, 2, f"FK Failures: {node.ik_error_count}", curses.color_pair(4) if node.ik_error_count > 0 else label_attr)
        
        # Refresh
        stdscr.refresh()
        
        # Check input
        try:
            c = stdscr.getch()
            if c == ord('q'):
                break
        except:
            pass
            
        # Sleep is handled by timeout(100) effectively, but let's be safe
        # time.sleep(0.05) 

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        curses.wrapper(draw_loop, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
