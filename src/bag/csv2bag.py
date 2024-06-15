import os

import pandas as pd
import rclpy
import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan


def create_bag_from_csv(folder_path, output_bag_path):
    # Initialize ROS2
    rclpy.init()

    # Create writer
    writer = rosbag2_py.SequentialWriter()

    storage_options = rosbag2_py._storage.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    writer.open(storage_options, converter_options)

    # Create topic information
    topic_info = rosbag2_py._storage.TopicMetadata(name='/scan', type='sensor_msgs/msg/LaserScan', serialization_format='cdr')
    writer.create_topic(topic_info)

    # Read all CSV files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith('.csv'):
            file_path = os.path.join(folder_path, filename)
            df = pd.read_csv(file_path)

            for index, row in df.iterrows():
                # Create LaserScan message
                scan = LaserScan()
                time_msg = Time()
                time_msg.sec = int(row['Timestamp'])
                time_msg.nanosec = int((row['Timestamp'] - int(row['Timestamp'])) * 1e9)
                scan.header.stamp = time_msg
                scan.header.frame_id = row['Frame_ID']
                scan.angle_min = row['Angle_Min']
                scan.angle_max = row['Angle_Max']
                scan.angle_increment = row['Angle_Increment']
                scan.time_increment = row['Time_Increment']
                scan.scan_time = row['Scan_Time']
                scan.range_min = row['Range_Min']
                scan.range_max = row['Range_Max']
                scan.ranges = [float(row['Distance'])]  # assuming 'Distance' is the range
                scan.intensities = [float(row['Intensity'])]  # assuming 'Intensity' is the intensity

                # Serialize and write message
                timestamp_ns = int(row['Timestamp'] * 1e9)
                writer.write('/scan', serialize_message(scan), timestamp_ns)

    # Cleanup
    rclpy.shutdown()


# Usage
folder_path = os.path.join(os.getcwd(), 'log', 'bagdata')
output_bag_path = os.path.join(folder_path, 'output')
create_bag_from_csv(folder_path, output_bag_path)
