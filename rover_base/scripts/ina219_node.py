#!/usr/bin/env python3
import rospy, smbus2
from std_msgs.msg import Float32MultiArray

I2C_BUS = 1
MUX_ADDR = 0x70
INA_ADDR = 0x40
MUX_CH_INA = 1

class INA219Node:
    def __init__(self):
        rospy.init_node("ina219_node")
        self.bus = smbus2.SMBus(I2C_BUS)
        self.pub = rospy.Publisher("power", Float32MultiArray, queue_size=10)
        self.select_mux(MUX_CH_INA)
        self.bus.write_word_data(INA_ADDR, 0x00, 0x399F)

    def select_mux(self, ch):
        self.bus.write_byte(MUX_ADDR, 1 << ch)

    def read_word(self, reg):
        raw = self.bus.read_word_data(INA_ADDR, reg)
        return ((raw & 0xFF) << 8) | (raw >> 8)

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.select_mux(MUX_CH_INA)
            bus_v_raw = (self.read_word(0x02) >> 3) * 4
            current_raw = self.read_word(0x04)
            v = bus_v_raw / 1000.0
            i = current_raw * 0.001
            self.pub.publish(Float32MultiArray(data=[v, i]))
            r.sleep()

if __name__ == "__main__":
    INA219Node().spin()
