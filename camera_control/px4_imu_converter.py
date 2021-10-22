import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined#, Timesync, TimesyncStatus
import builtin_interfaces.msg
from sensor_msgs.msg import Imu

class PX4ImuConverter(Node):

    def __init__(self):
        super().__init__('px4_imu_converter')
        self.subscription = self.create_subscription(SensorCombined, 'fmu/sensor_combined/out', self.imu_callback, 10)
        # self.subscription_timesync = self.create_subscription(Timesync, 'fmu/timesync/out', self.timesync_callback, 10)
        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.frame_id = 'imu'
        #self.time_offset = 0
        #self.time_scaler = 1
        #self.msg = Imu()
        #self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.frame_id
        seconds = int(msg.timestamp / 1000000000)
        nanoseconds = int(msg.timestamp % 1000000000)
        builtin_interfaces.msg.Time(sec=seconds, nanosec=nanoseconds)
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = float(msg.accelerometer_m_s2[2])
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = float(msg.gyro_rad[1])
        imu_msg.angular_velocity.z = float(msg.gyro_rad[2])
        #msg.accelerometer_clipping
        #imu_msg.linear_acceleration_covariance
        self.publisher.publish(imu_msg)
        #self.get_logger().info('Publishing: "%s"' % imu_msg.header.frame_id)

    # def timesync_callback(self, msg):
    #



def main(args=None):
    rclpy.init(args=args)

    imu_converter = PX4ImuConverter()

    rclpy.spin(imu_converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
