#!/usr/bin/env python3

import copy
import os

import rospy
from nav_msgs.msg import OccupancyGrid

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

map_path = os.path.join(log_path, 'map')

class MapSaver:
    def __init__(self):
        self.map_data = None
        self.previous_map_data = None
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.no_change_duration = rospy.get_param('~no_change_duration', 10)  # 데이터 변화 감지 시간 (초)
        self.last_change_time = rospy.get_time()
        self.output_file = rospy.get_param('~output_file', os.path.join(map_path, 'map.pgm'))  # 저장할 파일 이름

    def map_callback(self, data):
        self.map_data = data
        if self.previous_map_data is None:
            self.previous_map_data = copy.deepcopy(data)
            self.last_change_time = rospy.get_time()
            return
        
        if self.map_data.data != self.previous_map_data.data:
            self.previous_map_data = copy.deepcopy(self.map_data)
            self.last_change_time = rospy.get_time()
        else:
            if rospy.get_time() - self.last_change_time > self.no_change_duration:
                self.save_map()
                rospy.signal_shutdown('Map has been saved.')

    def save_map(self):
        with open(self.output_file, 'w') as f:
            for value in self.map_data.data:
                f.write(f'{value} ')
        rospy.loginfo(f'Map saved to {self.output_file}')


if __name__ == '__main__':
    rospy.init_node('map_saver')
    map_saver = MapSaver()
    rospy.spin()
