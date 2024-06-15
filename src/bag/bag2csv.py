import os

import pandas as pd
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def bag_to_csv(bag_file_path, output_folder):
    # Initialize ROS2
    rclpy.init()

    # Create reader
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py._storage.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader.open(storage_options, converter_options)

    # Get topic information
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Ensure output folder exists
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    data = {}

    while reader.has_next():
        (topic, msg_data, t) = reader.read_next()
        if topic not in data:
            data[topic] = []
        msg = deserialize_message(msg_data, LaserScan)
        data[topic].append({
            'Timestamp': t / 1e9,
            'Frame_ID': msg.header.frame_id,
            'Angle_Min': msg.angle_min,
            'Angle_Max': msg.angle_max,
            'Angle_Increment': msg.angle_increment,
            'Time_Increment': msg.time_increment,
            'Scan_Time': msg.scan_time,
            'Range_Min': msg.range_min,
            'Range_Max': msg.range_max,
            'Distance': ';'.join(map(str, msg.ranges)),
            'Intensity': ';'.join(map(str, msg.intensities))
        })

    # Write each topic to a CSV file
    for topic, records in data.items():
        df = pd.DataFrame(records)
        csv_file = os.path.join(output_folder, topic.replace('/', '_') + '.csv')
        df.to_csv(csv_file, index=False)

    # Cleanup
    rclpy.shutdown()

# Usage
bag_file_path = os.path.join(os.getcwd(), 'log', 'bagfile', 'rosbag2_2024_06_11-23_32_50_0.db3')
output_folder = os.path.join(os.getcwd(), 'log', 'csvfile')
bag_to_csv(bag_file_path, output_folder)
