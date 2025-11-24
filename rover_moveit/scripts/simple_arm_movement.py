#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_arm_to_pose(x, y, z, roll, pitch, yaw):
    moveit_commander.roscpp_initialize(sys.argv)
    arm_group = moveit_commander.MoveGroupCommander("arm")
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = roll
    pose_target.orientation.y = pitch
    pose_target.orientation.z = yaw
    pose_target.orientation.w = 1.0
    arm_group.set_pose_target(pose_target)
    arm_group.go(wait=True)
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    rospy.init_node("simple_arm_movement")
    move_arm_to_pose(0.5, 0.0, 0.5, 0.0, 0.0, 0.0)
    rospy.spin()
