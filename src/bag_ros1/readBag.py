import os

import bagpy
import pandas as pd
from bagpy import bagreader

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')
csv_folder = os.path.join(log_path, 'csvfile')
bag_path = os.path.join(log_path, 'bag', 'Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag')

# Create bagreader object
b = bagreader(bag_path)

# Get all topics
topics = b.topic_table

velmsgs = b.vel_data()
print("velmsgs:", end='')
print(velmsgs)

# # Function to save topic data to CSV
# def save_topic_to_csv(topic):
#     print(f"Processing topic: {topic}")
#     msg_csv = b.message_by_topic(topic)
#     if msg_csv:
#         df = pd.read_csv(msg_csv)
#         csv_file_path = os.path.join(csv_folder, topic.replace('/', '_') + '.csv')
#         df.to_csv(csv_file_path, index=False)
#         print(f"Saved {topic} to {csv_file_path}")
#     else:
#         print(f"No data for topic: {topic}")

# # Process and save each topic
# for topic in topics['Topics']:
#     save_topic_to_csv(topic)
