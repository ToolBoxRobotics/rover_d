#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

class ArmControl:
    def __init__(self):
        rospy.init_node("arm_control_node")
        self.joint_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
        rospy.Subscriber("/arm_command", Float64MultiArray, self.arm_command_cb)
        self.arm_state = [0.0] * 5  # 5 joints for the arm

    def arm_command_cb(self, msg):
        if len(msg.data) == 5:
            self.arm_state = msg.data
            self.send_arm_command()

    def send_arm_command(self):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        point.positions = self.arm_state
        point.time_from_start = rospy.Duration(1.0)
        traj.points.append(point)
        self.joint_pub.publish(traj)

if __name__ == "__main__":
    ArmControl()
    rospy.spin()
