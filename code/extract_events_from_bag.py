import rosbag
import csv
import numpy as np

## python -m debugpy --listen 5678 --wait-for-client ./extract_events_from_bag.py


event_data_array = []
fieldnames = ['x','y','t','polarity']
with rosbag.Bag('C:/Users/sudes/Documents/GitHub/Computer_Vision/code/sudeshna_panorama02.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/cam0/events']):
        for i in range(len(msg.events)):
            if np.abs(msg.events[i].ts.to_sec() - 0.210521) <= 0.01: 
                event_data_array.append({'x' : msg.events[i].x , 'y' : msg.events[i].y , 't' : msg.events[i].ts.to_sec() , 'polarity' : msg.events[i].polarity})


# write into the csv file
with open('C:/Users/sudes/Documents/GitHub/Computer_Vision/code/event_data.csv', mode='w') as csv_file:
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    for i in range(len(event_data_array)):
        writer.writerow({'x':event_data_array[i].get('x'),'y':event_data_array[i].get('y'),'t':event_data_array[i].get('t'),'polarity':event_data_array[i].get('polarity')})
