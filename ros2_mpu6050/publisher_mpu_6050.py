import smbus
from time import sleep
from mpu6050 import mpu6050
import rclpy
from rclpy.node import Node

from ros2_mpu6050_interfaces.msg import Accel, Gyro

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
        self.accel_publisher_ = self.create_publisher(Accel, '/mpu6050/accel', 10)
        self.gyro_publisher_ = self.create_publisher(Gyro, '/mpu6050/gyro', 10)
        self.sensor = mpu6050(Device_Address)
        self.accel_timer_ = self.create_timer(0.1, self.publish_accel)
        self.gyro_timer_ = self.create_timer(0.1, self.publish_gyro)


    def publish_accel(self):
        accelerometer_data = self.sensor.get_accel_data()
        msg = Accel()
        msg.x = accelerometer_data['x']
        msg.y = accelerometer_data['y']
        msg.z = accelerometer_data['z']
        self.accel_publisher_.publish(msg)
        self.get_logger().info('Accel Data: "%s"' % accelerometer_data)


    def publish_gyro(self):
        gyro_data = self.sensor.get_gyro_data()
        msg = Gyro()
        msg.x = gyro_data['x']
        msg.y = gyro_data['y']
        msg.z = gyro_data['z']
        self.gyro_publisher_.publish(msg)
        self.get_logger().info('Gyro Data: "%s"' % gyro_data)



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
