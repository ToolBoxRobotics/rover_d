#!/usr/bin/env python3
import rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class CmdVelWatchdog:
    def __init__(self):
        rospy.init_node("cmd_vel_watchdog")
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.last_cmd = time.time()
        self.pub_speed = rospy.Publisher("wheel_speed_cmd", Float32MultiArray, queue_size=1)
        self.pub_steer = rospy.Publisher("steering_cmd", Float32MultiArray, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.cb)

    def cb(self, _):
        self.last_cmd = time.time()

    def spin(self):
        r = rospy.Rate(20)
        warned = False
        while not rospy.is_shutdown():
            if time.time() - self.last_cmd > self.timeout:
                if not warned:
                    rospy.logwarn("cmd_vel timeout -> STOP")
                    warned = True
                self.pub_speed.publish(Float32MultiArray(data=[0.0] * 6))
                self.pub_steer.publish(Float32MultiArray(data=[0.0] * 4))
            else:
                warned = False
            r.sleep()

if __name__ == "__main__":
    CmdVelWatchdog().spin()
