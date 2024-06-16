import os
import shutil

import pandas as pd
import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import (ConverterOptions, SequentialWriter, StorageOptions,
                        TopicMetadata)
from sensor_msgs.msg import LaserScan

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

def csv_to_bag(csv_file_path, bag_file_path):
    # Initialize ROS2
    rclpy.init()

    # Remove existing directory if it exists
    if os.path.exists(bag_file_path):
        shutil.rmtree(bag_file_path)

    # Step 1: Parse the CSV File
    df = pd.read_csv(csv_file_path)

    # Step 2: Create ROS2 Messages
    scan_msgs = []
    for index, row in df.iterrows():
        scan_msg = LaserScan()
        scan_msg.header.stamp.sec = int(row['Timestamp'])
        scan_msg.header.stamp.nanosec = int((row['Timestamp'] - int(row['Timestamp'])) * 1e9)
        scan_msg.header.frame_id = row['Frame_ID']
        scan_msg.angle_min = float(row['Angle_Min'])
        scan_msg.angle_max = float(row['Angle_Max'])
        scan_msg.angle_increment = float(row['Angle_Increment'])
        scan_msg.time_increment = float(row['Time_Increment'])
        scan_msg.scan_time = float(row['Scan_Time'])
        scan_msg.range_min = float(row['Range_Min'])
        scan_msg.range_max = float(row['Range_Max'])
        scan_msg.ranges = [float(x) for x in row['Distance'].split(';')]
        scan_msg.intensities = [float(x) for x in row['Intensity'].split(';')]
        scan_msgs.append(scan_msg)

    # Step 3: Write to ROS2 Bag File
    storage_options = StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # Create topic
    topic_name = '/scan'
    topic_type = 'sensor_msgs/msg/LaserScan'
    serialization_format = 'cdr'
    topic_metadata = TopicMetadata(name=topic_name, type=topic_type, serialization_format=serialization_format)
    writer.create_topic(topic_metadata)

    for msg in scan_msgs:
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        writer.write(topic_name, serialize_message(msg), int(timestamp * 1e9))

    # Finalize
    writer.close()
    rclpy.shutdown()
    print(f"Converted CSV data to ROS2 bag file at {bag_file_path}")

if __name__ == '__main__':
    csv_path = os.path.join(log_path, 'csvfile', '_scan.csv')
    output_bag_path = os.path.join(log_path, 'bagfile', 'restore')
    csv_to_bag(csv_path, output_bag_path)
