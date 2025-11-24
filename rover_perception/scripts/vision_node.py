#!/usr/bin/env python3
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionNode:
  def __init__(self):
    rospy.init_node("vision_node")
    self.bridge = CvBridge()
    rospy.Subscriber("/camera/rgb/image_raw", Image, self.cb)

  def cb(self, msg):
    img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    # Placeholder for OpenCV-based object detection or other perception tasks

if __name__ == "__main__":
  VisionNode()
  rospy.spin()
