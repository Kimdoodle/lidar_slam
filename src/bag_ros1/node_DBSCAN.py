#!/usr/bin/env python3

import signal
import sys

import rospy
from sensor_msgs.msg import LaserScan
from train import compute_DBSCAN

processTime = 0.0
removePoint = 0
count = 0

cluster = True

def callback(data):
    global processTime, removePoint, count
    eps_ratio = rospy.get_param('~eps_ratio', 0.5)
    remains = rospy.get_param('~remains', 5)
    
    if cluster:
        modified_msg, processing_time, removed_count = compute_DBSCAN(data, eps_ratio, remains, False)
    else:
        modified_msg, processing_time, removed_count = data, 0.0, 0
    
    # 결과를 퍼블리시
    pub.publish(modified_msg)
    print(f"Processing time: {processing_time}, Removed points: {removed_count}")
    processTime += processing_time
    removePoint += removed_count
    count += 1

def shutdown_handler(signal, frame):
    if count > 0:
        avg_process_time = processTime / count
        avg_removed_points = removePoint / count
    else:
        avg_process_time = 0
        avg_removed_points = 0

    print("\nShutting down...")
    print(f"Average processing time: {avg_process_time:.4f} seconds")
    print(f"Average removed points: {avg_removed_points:.4f}")
    print(f"Total scans processed: {count}")
    
    sys.exit(0)

def listener():
    global pub
    
    # ROS 노드 초기화
    rospy.init_node('scan_listener', anonymous=True)
    
    # 퍼블리셔 설정
    pub = rospy.Publisher('/scan_modified', LaserScan, queue_size=10)
    
    # /scan 토픽을 구독하고 callback 함수 지정
    rospy.Subscriber('/scan', LaserScan, callback)
    
    # 종료 핸들러 설정
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    
    # 콜백 함수가 호출될 때까지 대기
    rospy.spin()

if __name__ == '__main__':
    listener()
