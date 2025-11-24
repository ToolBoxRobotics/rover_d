#!/usr/bin/env python3
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectDetectionNode:
  def __init__(self):
    rospy.init_node("object_detection_node")
    self.bridge = CvBridge()
    rospy.Subscriber("/camera/rgb/image_raw", Image, self.cb)

  def cb(self, msg):
    img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    # Here, you can implement detection logic for obstacles or objects
    # Example: applying a simple color filter to detect a color
    lower = (0, 0, 100)
    upper = (100, 100, 255)
    mask = cv2.inRange(img, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    # Publish the results or visualize with OpenCV

if __name__ == "__main__":
  ObjectDetectionNode()
  rospy.spin()
