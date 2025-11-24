#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class SixWheelAckermann:
    def __init__(self):
        rospy.init_node("ackermann_kinematics")
        self.L = rospy.get_param("~wheelbase", 0.50)
        self.W = rospy.get_param("~track_width", 0.40)
        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 35.0)
        self.pub_speed = rospy.Publisher("wheel_speed_cmd", Float32MultiArray, queue_size=1)
        self.pub_steer = rospy.Publisher("steering_cmd", Float32MultiArray, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

    def clamp(self, v, lo, hi): return max(lo, min(hi, v))

    def compute(self, vx, wz):
        if abs(wz) < 1e-6:
            return [0,0,0,0], [self.clamp(vx/self.max_speed,-1,1)]*6
        R = vx / wz
        L = self.L
        halfW = self.W / 2.0
        delta_FL = math.atan2(L, (R - halfW))
        delta_FR = math.atan2(L, (R + halfW))
        delta_RL = -delta_FL
        delta_RR = -delta_FR
        clamp_rad = math.radians(self.max_steer_deg)
        angles = [self.clamp(delta_FL,-clamp_rad,clamp_rad),
                  self.clamp(delta_FR,-clamp_rad,clamp_rad),
                  self.clamp(delta_RL,-clamp_rad,clamp_rad),
                  self.clamp(delta_RR,-clamp_rad,clamp_rad)]
        R_FL = math.hypot(R-halfW, L)
        R_FR = math.hypot(R+halfW, L)
        R_ML = abs(R-halfW)
        R_MR = abs(R+halfW)
        radii = [R_FL, R_ML, R_FL, R_FR, R_MR, R_FR]
        wheel_speeds = [wz*r for r in radii]
        maxv = max(abs(v) for v in wheel_speeds) or 1.0
        if maxv > self.max_speed:
            wheel_speeds = [v/maxv*self.max_speed for v in wheel_speeds]
        speeds_norm = [self.clamp(v/self.max_speed,-1,1) for v in wheel_speeds]
        return angles, speeds_norm

    def cmd_cb(self, msg):
        angles, speeds = self.compute(msg.linear.x, msg.angular.z)
        self.pub_steer.publish(Float32MultiArray(data=[math.degrees(a) for a in angles]))
        self.pub_speed.publish(Float32MultiArray(data=speeds))

if __name__ == "__main__":
    SixWheelAckermann()
    rospy.spin()
