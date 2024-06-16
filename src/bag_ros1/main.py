import os
import sys
import time

import rosbag
from train import compute_DBSCAN

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

bagname = 'Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag'
EPS_RATIO = 95
STRIDE = 3
bagname2 = f'compressed_{EPS_RATIO}_{STRIDE}.bag'
bag_path = os.path.join(log_path, 'bag', bagname)
bag_path2 = os.path.join(log_path, 'bag', bagname2)

calTime = 0.
removed = 0
count = 0

with rosbag.Bag(bag_path2, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(bag_path).read_messages():
        if topic == '/scan':
            # <class 'tmpb5seuoht._sensor_msgs__LaserScan'>
            msg, time2, remove\
                = compute_DBSCAN(msg, eps_ratio=EPS_RATIO, stride=STRIDE)
            
            calTime += time2
            removed += remove
            count += 1
            if count % 1000 == 1:
                print(f"{count-1} index completed.")
        outbag.write(topic, msg, t)
print(f"total scan msg: {count}")
print(f"average removed data: {removed/count}")
print("time used for DBSCAN :", calTime/count)