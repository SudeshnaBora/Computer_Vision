import roslib;

import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class BagActivities():
    def __init__(self):
        self.bridge = CvBridge()


if __name__ == '__main__':
    try:
        image_creator = ImageCreator()
		#image_creator.extract_events('/cam0/events')
    except rospy.ROSInterruptException:
        pass

