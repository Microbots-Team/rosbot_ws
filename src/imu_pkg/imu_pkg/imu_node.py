#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class CalibratedMPU9250:
    def __init__(self, bus=1, address=0x68):
        import smbus
        self.bus = smbus.SMBus(bus)
        self.address = address
        
        # Initialize IMU
        self._initialize_imu()
        self.calibration_samples = 200
        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrate()

    def _initialize_imu(self):
        # Wake up device
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        # Accel config (±16G)
        self.bus.write_byte_data(self.address, 0x1C, 0x18)
        # Gyro config (±2000dps)
        self.bus.write_byte_data(self.address, 0x1B, 0x18)

    def _read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg+1)
        val = (high << 8) + low
        return val - 65536 if val > 32767 else val

    def calibrate(self):
        accel_samples = []
        gyro_samples = []
        
        print("Calibrating IMU - keep device stationary...")
        for _ in range(self.calibration_samples):
            accel_samples.append(self.get_raw_accel())
            gyro_samples.append(self.get_raw_gyro())
        
        self.accel_bias = np.mean(accel_samples, axis=0)
        self.gyro_bias = np.mean(gyro_samples, axis=0)
        print(f"Calibration complete:\nAccel Bias: {self.accel_bias}\nGyro Bias: {self.gyro_bias}")

    def get_raw_accel(self):
        return [
            self._read_word_2c(0x3B),
            self._read_word_2c(0x3D),
            self._read_word_2c(0x3F)
        ]

    def get_raw_gyro(self):
        return [
            self._read_word_2c(0x43),
            self._read_word_2c(0x45),
            self._read_word_2c(0x47)
        ]

    def get_accel(self):
        raw = self.get_raw_accel()
        return [
            (raw[0] - self.accel_bias[0]) / 2048.0 * 9.80665,  # m/s²
            (raw[1] - self.accel_bias[1]) / 2048.0 * 9.80665,
            (raw[2] - self.accel_bias[2]) / 2048.0 * 9.80665
        ]

    def get_gyro(self):
        raw = self.get_raw_gyro()
        return [
            (raw[0] - self.gyro_bias[0]) / 16.4 * math.pi / 180.0,  # rad/s
            (raw[1] - self.gyro_bias[1]) / 16.4 * math.pi / 180.0,
            (raw[2] - self.gyro_bias[2]) / 16.4 * math.pi / 180.0
        ]

class CalibratedIMUNode(Node):
    def __init__(self):
        super().__init__('calibrated_imu_node')
        
        # Configure QoS
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.publisher = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.timer = self.create_timer(0.02, self.publish_data)  # 50Hz
        
        try:
            self.imu = CalibratedMPU9250(bus=1, address=0x68)
            self.get_logger().info("IMU initialized with calibration")
        except Exception as e:
            self.get_logger().fatal(f"IMU init failed: {str(e)}")
            raise

    def publish_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Get calibrated data
        accel = self.imu.get_accel()
        gyro = self.imu.get_gyro()

        # Populate message
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        # Covariance matrices (adjust based on your sensor specs)
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0, 
            0.0, 0.0, 0.04
        ]
        
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CalibratedIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
