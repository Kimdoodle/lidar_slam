#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from train import compute_DBSCAN


def callback(data):
    modified_msg, processing_time, removed_count = compute_DBSCAN(data, 50, 2, True)
    
    # 결과를 퍼블리시
    pub.publish(modified_msg)
    print(f"Processing time: {processing_time}, Removed points: {removed_count}")

def listener():
    global pub
    
    # ROS 노드 초기화
    rospy.init_node('scan_listener', anonymous=True)
    
    # 퍼블리셔 설정
    pub = rospy.Publisher('/scan_modified', LaserScan, queue_size=10)
    
    # /scan 토픽을 구독하고 callback 함수 지정
    rospy.Subscriber('/scan', LaserScan, callback)
    
    # 콜백 함수가 호출될 때까지 대기
    rospy.spin()

if __name__ == '__main__':
    listener()
