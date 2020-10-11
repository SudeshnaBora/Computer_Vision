import roslib

import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
# does not work in windows 
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import csv 
import numpy as np
import pdb
import pandas as pd

"""
The class where all bag file related activities are defined. 
"""




class bag_activities():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_path = '/tmp/image29092020/'
        self.events_path = '/tmp/event_data01102020.csv'
        self.bag_path = '/tmp/shapes_rotation.bag'
		
    def store_images(self, topic_name):
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
                    cv2.imwrite(self.image_path + image_name, cv_image) #Save;
					
    def extract_events(self, topic_name, timestamp, threshold,bag_file):
        event_data_array = []
        fieldnames = ['x','y','t','polarity']
		if bag_file:
            with rosbag.Bag(self.bag_path, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=[topic_name]):
                    for i in range(len(msg.events)):
                        if np.abs(msg.events[i].ts.to_sec() - timestamp) <= threshold: 
                            event_data_array.append({'x' : msg.events[i].x , 'y' : msg.events[i].y , 't' : msg.events[i].ts.to_sec() , 'polarity' : msg.events[i].polarity})
        else:
            events_frame = pd.read_csv('events.txt', sep = " ", header = None)
            for index, rows in events_frame.iterrows():
                if np.abs(rows[0] - timestamp) <= threshold:
                    event_data_array.append({'x' : rows[1] , 'y' : rows[2] , 't' : rows[0] , 'polarity' : rows[3]})
		    


        # write into the csv file
            with open(self.events_path, mode='w') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()
                for i in range(len(event_data_array)):
                    writer.writerow({'x':event_data_array[i].get('x'),'y':event_data_array[i].get('y'),'t':event_data_array[i].get('t'),'polarity':event_data_array[i].get('polarity')})

	

if __name__ == '__main__':
    try:
        bag_activities = bag_activities()
        bag_activities.extract_events('/dvs/events',1.217811001,0.01,False)
        #bag_activities.store_images('/cam0/image_corrupted')
    except rospy.ROSInterruptException:
        pass

