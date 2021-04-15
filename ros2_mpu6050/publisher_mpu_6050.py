import smbus
from time import sleep
from mpu6050 import mpu6050
import rclpy
from rclpy.node import Node

from ros2_mpu6050_interfaces.msg import Accel

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
Device_Address = 0x68

class MPU6050Publisher(Node):

    def __init__(self):
        super().__init__('mpu6050_publisher')
        self.publisher_ = self.create_publisher(Accel, '/mpu6050/accel', 10)
        sensor = mpu6050(Device_Address)
        accelerometer_data = sensor.get_accel_data()
        msg = Accel()
        msg.x = accelerometer_data['x']
        self.publisher_.publish(msg)
        self.get_logger().info('Data: "%s"' % accelerometer_data)



def main(args=None):
    rclpy.init(args=args)

    mpu6050_publisher = MPU6050Publisher()

    rclpy.spin(mpu6050_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpu6050_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
