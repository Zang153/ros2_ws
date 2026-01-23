import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleImu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('px4_imu_reader')

        # Configure QoS profile to match PX4's default settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the VehicleImu topic
        # Note: In recent PX4 versions (v1.14+), sensor data is typically published 
        # to /fmu/out/vehicle_imu or similar paths.
        self.subscription = self.create_subscription(
            VehicleImu,
            '/fmu/out/vehicle_imu',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('IMU Subscriber Node has been started.')

    def listener_callback(self, msg):
        # Print the IMU data
        # accel: [x, y, z] in m/s^2
        # gyro: [x, y, z] in rad/s
        self.get_logger().info(
            f'\n\tAccel: [{msg.accelerometer[0]:.2f}, {msg.accelerometer[1]:.2f}, {msg.accelerometer[2]:.2f}]'
            f'\n\tGyro:  [{msg.gyro[0]:.2f}, {msg.gyro[1]:.2f}, {msg.gyro[2]:.2f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    
    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
