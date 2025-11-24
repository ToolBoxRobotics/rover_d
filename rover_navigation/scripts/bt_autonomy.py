#!/usr/bin/env python3
import rospy, py_trees, actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveToPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, pose):
        super().__init__(name)
        self.pose = pose
        self.sent = False
        self.client = None

    def setup(self, timeout):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(10))
        return True

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            g = MoveBaseGoal()
            g.target_pose = self.pose
            self.client.send_goal(g)
            self.sent = True
            return py_trees.common.Status.RUNNING
        s = self.client.get_state()
        if s == 3:
            return py_trees.common.Status.SUCCESS
        if s in [4, 5, 8]:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

def make_pose(x, y, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    return p

def create_tree():
    root = py_trees.composites.Sequence("Mission")
    root.add_children([MoveToPose("WP1", make_pose(1, 0)), MoveToPose("WP2", make_pose(2, 1))])
    return root

if __name__ == "__main__":
    rospy.init_node("bt_autonomy")
    bt = py_trees.trees.BehaviourTree(create_tree())
    bt.setup(timeout=15)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        bt.tick()
        r.sleep()
