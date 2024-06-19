# modify rosbag, cherry pick topics
import os

import rosbag

# Define paths
file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

bagname = 'final.bag'
bag_path = os.path.join(log_path, 'bag', bagname)
bag_path2 = os.path.join(log_path, 'bag', 'test_output.bag')

if __name__ == '__main__':
    with rosbag.Bag(bag_path, 'r') as inbag, rosbag.Bag(bag_path2, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            if topic.startswith('/scan') | topic.startswith('/rosout'):
                outbag.write(topic, msg, t)
