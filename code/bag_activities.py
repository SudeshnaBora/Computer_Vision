import roslib;

import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import csv 
import numpy as np

 """
    The class where all bag file related activities are defined. 
 """




class bag_activities():
    def __init__(self):
        self.bridge = CvBridge()
		self.image_path = 'C:/Users/sudes/Documents/GitHub/Computer_Vision/code/bag_image/'
        self.events_path = 'C:/Users/sudes/Documents/GitHub/Computer_Vision/code/event_data.csv'
        self.bag_path = 'C:/Users/sudes/Documents/GitHub/Computer_Vision/code/sudeshna_panorama02.bag'
		
	def store_images(topic_name):
	"""
	  This stores the gaussian blurred images (png) from the given topic onto a folder with naming convention of the timestamp of procurement
	  
	  Arguments:
	  
	  topic_name : The name of the topic where the images are stored
	"""
        with rosbag.Bag(self.bag_path, 'r') as bag: #bag file to be read;
            for topic,msg,t in bag.read_messages():
                if topic == topic_name: #topic of the image;
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" %  msg.header.stamp.to_sec()
                                                 #%.6f means that there are 6 digits after the decimal point, which can be modified according to the accuracy;
                    image_name = timestr+ ".png" #Image Naming: Timestamp.png
                    cv2.imwrite(self.path + image_name, cv_image) #Save;
	
	def extract_events(topic_name,base_timestamp,threshold):
	"""
	   Extracts events between a particular time threshold of an image. It then writes into a csv file.
	   
	   Arguments:
	   
	   topic_name: the name of the topic where the event is stored
	   base_timestamp: the timestamp (in sec) of the image.
	   threshold: the events to be captured needs to be within this threshold from the image timestamp 
	   
	"""
	    event_data_array = []
		fieldnames = ['x','y','t','polarity']
		with rosbag.Bag(self.bag_path, 'r') as bag:
		    for topic, msg, t in bag.read_messages(topics=[topic_name]):
			    # source code of class : https://github.com/uzh-rpg/rpg_dvs_ros/tree/master/dvs_msgs/msg
			    for i in range(len(msg.events)):
				    if np.abs(msg.events[i].ts.to_sec() - base_timestamp) <= threshold: 
					    event_data_array.append({'x' : msg.events[i].x , 'y' : msg.events[i].y , 't' : msg.events[i].ts.to_sec() , 'polarity' : msg.events[i].polarity})


        # write into the csv file
        with open(self.events_path, mode='w') as csv_file:
		    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
			writer.writeheader()
			for i in range(len(event_data_array)):
			    writer.writerow({'x':event_data_array[i].get('x'),'y':event_data_array[i].get('y'),'t':event_data_array[i].get('t'),'polarity':event_data_array[i].get('polarity')})

if __name__ == '__main__':
    try:
        image_creator = ImageCreator('/cam0/images_corrupted')
		#image_creator.extract_events('/cam0/events',0.210521,0.01)
    except rospy.ROSInterruptException:
        pass

