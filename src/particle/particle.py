import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Turtlebot:
  err = 0

  def __init__(self):
    self.scan_sub = rospy.Subscriber('scan', 1, self.scan_cb)



  def scan_cb(self, msg):
    # reads the image from camera topic and transform to OpenCV format
    image = msg

    print msg

    #cv2.imshow("window", image)
    #cv2.waitKey(3)

rospy.init_node('turtlebot_tracker')
turtlebot = Turtlebot()
rospy.spin()
