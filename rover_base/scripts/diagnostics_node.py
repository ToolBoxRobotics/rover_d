#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, NavSatFix, AnyMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class RoverDiagnostics:
    def __init__(self):
        rospy.init_node("rover_diagnostics")
        self.last_imu = 0
        self.last_gps = 0
        self.last_power = 0
        self.last_cmd = 0
        self.power = None
        rospy.Subscriber("imu/data_raw", Imu, lambda _: setattr(self, 'last_imu', time.time()))
        rospy.Subscriber("fix", NavSatFix, lambda _: setattr(self, 'last_gps', time.time()))
        rospy.Subscriber("power", Float32MultiArray, self.power_cb)
        rospy.Subscriber("cmd_vel", AnyMsg, lambda _: setattr(self, 'last_cmd', time.time()))
        self.pub = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=1)

    def power_cb(self, msg):
        self.last_power = time.time()
        self.power = msg.data

    def cpu_temp(self):
        try:
            return float(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
        except:
            return -1

    def mk(self, name, level, message, vals):
        s = DiagnosticStatus(name=name, level=level, message=message)
        for k, v in vals.items():
            s.values.append(KeyValue(key=str(k), value=str(v)))
        return s

    def spin(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            now = time.time()
            arr = DiagnosticArray()
            arr.header.stamp = rospy.Time.now()
            imu_ok = now - self.last_imu < 1.0
            arr.status.append(self.mk("IMU", 0 if imu_ok else 2, "OK" if imu_ok else "No IMU", {"age": now - self.last_imu}))
            gps_ok = now - self.last_gps < 2.0
            arr.status.append(self.mk("GPS", 0 if gps_ok else 1, "OK" if gps_ok else "No GPS", {"age": now - self.last_gps}))
            if self.power:
                v, i = self.power
                lvl = 0
                msg = "OK"
                if v < 10.5:
                    lvl = 2
                    msg = "LOW BATTERY"
                arr.status.append(self.mk("Battery", lvl, msg, {"voltage": v, "current": i}))
            temp = self.cpu_temp()
            lvl = 0
            msg = "Normal"
            if temp > 70:
                lvl = 1
                msg = "HOT"
            if temp > 80:
                lvl = 2
                msg = "OVERHEAT"
            arr.status.append(self.mk("CPU Temperature", lvl, msg, {"cpu_temp_c": temp}))
            cmd_ok = now - self.last_cmd < 0.5
            arr.status.append(self.mk("Command Stream", 0 if cmd_ok else 1, "OK" if cmd_ok else "cmd_vel timeout", {"age": now - self.last_cmd}))
            self.pub.publish(arr)
            r.sleep()

if __name__ == "__main__":
    RoverDiagnostics().spin()
