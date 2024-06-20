#!/usr/bin/env python3

import copy
import os
import numpy as np
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
        width = self.map_data.info.width
        height = self.map_data.info.height
        max_value = 255

        # OccupancyGrid 데이터 변환 (-1: unknown, 0: free, 100: occupied)
        image_data = np.array(self.map_data.data).reshape((height, width))
        image_data = np.where(image_data == -1, 205, image_data)  # unknown -> 205 (중간 값)
        image_data = np.where(image_data == 0, 255, image_data)   # free -> 255 (white)
        image_data = np.where(image_data == 100, 0, image_data)  # occupied -> 0 (black)

        with open(self.output_file, 'wb') as f:
            # 헤더 작성
            f.write(b'P5\n')
            f.write(f'{width} {height}\n'.encode())
            f.write(f'{max_value}\n'.encode())

            # 이미지 데이터 작성
            image_data = image_data.astype(np.uint8)
            image_data.tofile(f)
        
        rospy.loginfo(f'Map saved to {self.output_file}')


if __name__ == '__main__':
    rospy.init_node('map_saver')
    map_saver = MapSaver()
    rospy.spin()
