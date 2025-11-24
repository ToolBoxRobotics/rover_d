#!/usr/bin/env python3
import rospy, smbus2, math
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

I2C_BUS = 1
MUX_ADDR = 0x70
IMU_ADDR = 0x68
MUX_CH_IMU = 0

class MPU6050Node:
    def __init__(self):
        rospy.init_node("mpu6050_node")
        self.bus = smbus2.SMBus(I2C_BUS)
        self.pub = rospy.Publisher("imu/data_raw", Imu, queue_size=10)
        self.select_mux(MUX_CH_IMU)
        self.bus.write_byte_data(IMU_ADDR, 0x6B, 0x00)

    def select_mux(self, ch):
        self.bus.write_byte(MUX_ADDR, 1 << ch)

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(IMU_ADDR, reg)
        low = self.bus.read_byte_data(IMU_ADDR, reg + 1)
        val = (high << 8) + low
        return val - 65536 if val >= 0x8000 else val

    def spin(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.select_mux(MUX_CH_IMU)
            ax = self.read_word_2c(0x3B) / 16384.0
            ay = self.read_word_2c(0x3D) / 16384.0
            az = self.read_word_2c(0x3F) / 16384.0
            gx = self.read_word_2c(0x43) / 131.0 * math.pi / 180.0
            gy = self.read_word_2c(0x45) / 131.0 * math.pi / 180.0
            gz = self.read_word_2c(0x47) / 131.0 * math.pi / 180.0

            msg = Imu()
            msg.header = Header(stamp=rospy.Time.now(), frame_id="imu_link")
            msg.linear_acceleration.x = ax * 9.81
            msg.linear_acceleration.y = ay * 9.81
            msg.linear_acceleration.z = az * 9.81
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            self.pub.publish(msg)
            r.sleep()

if __name__ == "__main__":
    MPU6050Node().spin()
