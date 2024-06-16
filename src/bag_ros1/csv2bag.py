import os

import pandas as pd
import rosbag
import rospy
# from asctec_msgs.msg import IMUCalcData, LLStatus
from diagnostic_msgs.msg import DiagnosticArray
from dynamic_reconfigure.msg import Config, ConfigDescription
from geometry_msgs.msg import TransformStamped
from gps_common.msg import GPSFix
# from mav_msgs.msg import Height
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

csv_folder = os.path.join(log_path, 'csvfile')
bag_path = os.path.join(log_path, 'bag', 'restored.bag')

def create_rosbag_from_csv(csv_folder_path, output_bag_path):
    bag = rosbag.Bag(output_bag_path, 'w')
    
    # Helper function to create a standard header
    def create_header(seq, stamp, frame_id):
        header = Header()
        header.seq = seq
        header.stamp = rospy.Time.from_sec(stamp)
        header.frame_id = frame_id
        return header
    
    # Process each CSV file
    for csv_file in os.listdir(csv_folder_path):
        topic_name = '/' + csv_file.replace('.csv', '').replace('_', '/')
        file_path = os.path.join(csv_folder_path, csv_file)
        df = pd.read_csv(file_path)
        
        for index, row in df.iterrows():
            msg = None  # Initialize msg variable
            # Create messages based on the topic type
            if 'imu' in topic_name:
                msg = Imu()
                msg.header = create_header(row['seq'], row['stamp'], row['frame_id'])
                msg.orientation.x = row['orientation_x']
                msg.orientation.y = row['orientation_y']
                msg.orientation.z = row['orientation_z']
                msg.orientation.w = row['orientation_w']
                msg.angular_velocity.x = row['angular_velocity_x']
                msg.angular_velocity.y = row['angular_velocity_y']
                msg.angular_velocity.z = row['angular_velocity_z']
                msg.linear_acceleration.x = row['linear_acceleration_x']
                msg.linear_acceleration.y = row['linear_acceleration_y']
                msg.linear_acceleration.z = row['linear_acceleration_z']
            elif 'scan' in topic_name:
                msg = LaserScan()
                msg.header = create_header(row['seq'], row['stamp'], row['frame_id'])
                msg.angle_min = row['angle_min']
                msg.angle_max = row['angle_max']
                msg.angle_increment = row['angle_increment']
                msg.time_increment = row['time_increment']
                msg.scan_time = row['scan_time']
                msg.range_min = row['range_min']
                msg.range_max = row['range_max']
                msg.ranges = list(map(float, row['ranges'].strip('[]').split(',')))
                msg.intensities = list(map(float, row['intensities'].strip('[]').split(',')))
            elif 'solution' in topic_name:
                msg = Odometry()
                msg.header = create_header(row['seq'], row['stamp'], row['frame_id'])
                msg.pose.pose.position.x = row['position_x']
                msg.pose.pose.position.y = row['position_y']
                msg.pose.pose.position.z = row['position_z']
                msg.pose.pose.orientation.x = row['orientation_x']
                msg.pose.pose.orientation.y = row['orientation_y']
                msg.pose.pose.orientation.z = row['orientation_z']
                msg.pose.pose.orientation.w = row['orientation_w']
                msg.twist.twist.linear.x = row['linear_velocity_x']
                msg.twist.twist.linear.y = row['linear_velocity_y']
                msg.twist.twist.linear.z = row['linear_velocity_z']
                msg.twist.twist.angular.x = row['angular_velocity_x']
                msg.twist.twist.angular.y = row['angular_velocity_y']
                msg.twist.twist.angular.z = row['angular_velocity_z']
            # Add additional topic handling as needed based on your topics
            
            if msg is not None:
                bag.write(topic_name, msg, msg.header.stamp)
    
    bag.close()
    print(f"New rosbag created at {output_bag_path}")

create_rosbag_from_csv(csv_folder, bag_path)



# def create_laserscan_message(row):
#     header = Header()
#     stamp_secs = int(row['%time']) // 10**9
#     stamp_nsecs = int(row['%time']) % 10**9
#     header.stamp = rospy.Time(secs=stamp_secs, nsecs=stamp_nsecs)
#     header.frame_id = row['field.header.frame_id']
    
#     scan = LaserScan()
#     scan.header = header
#     scan.angle_min = float(row['field.angle_min'])
#     scan.angle_max = float(row['field.angle_max'])
#     scan.angle_increment = float(row['field.angle_increment'])
#     scan.time_increment = float(row['field.time_increment'])
#     scan.scan_time = float(row['field.scan_time'])
#     scan.range_min = float(row['field.range_min'])
#     scan.range_max = float(row['field.range_max'])
    
#     ranges_columns = [f'field.ranges{i}' for i in range(1040)]
#     ranges = [float(row[col]) if row[col] != 'inf' else np.inf for col in ranges_columns]
#     scan.ranges = ranges
    
#     return scan

# def create_odometry_message(row):
#     header = Header()
#     stamp_secs = int(row['%time']) // 10**9
#     stamp_nsecs = int(row['%time']) % 10**9
#     header.stamp = rospy.Time(secs=stamp_secs, nsecs=stamp_nsecs)
#     header.frame_id = row['field.header.frame_id']

#     odom = Odometry()
#     odom.header = header
#     odom.child_frame_id = row['field.child_frame_id']
    
#     odom.pose.pose.position.x = float(row['field.pose.pose.position.x'])
#     odom.pose.pose.position.y = float(row['field.pose.pose.position.y'])
#     odom.pose.pose.position.z = float(row['field.pose.pose.position.z'])
#     odom.pose.pose.orientation.x = float(row['field.pose.pose.orientation.x'])
#     odom.pose.pose.orientation.y = float(row['field.pose.pose.orientation.y'])
#     odom.pose.pose.orientation.z = float(row['field.pose.pose.orientation.z'])
#     odom.pose.pose.orientation.w = float(row['field.pose.pose.orientation.w'])
    
#     odom.twist.twist.linear.x = float(row['field.twist.twist.linear.x'])
#     odom.twist.twist.linear.y = float(row['field.twist.twist.linear.y'])
#     odom.twist.twist.linear.z = float(row['field.twist.twist.linear.z'])
#     odom.twist.twist.angular.x = float(row['field.twist.twist.angular.x'])
#     odom.twist.twist.angular.y = float(row['field.twist.twist.angular.y'])
#     odom.twist.twist.angular.z = float(row['field.twist.twist.angular.z'])
    
#     return odom

# def create_transform_message(stamp, parent_frame, child_frame, translation, rotation):
#     transform = TransformStamped()
#     transform.header.stamp = stamp
#     transform.header.frame_id = parent_frame
#     transform.child_frame_id = child_frame
#     transform.transform.translation = Vector3(*translation)
#     transform.transform.rotation = Quaternion(*rotation)
#     return transform

# def csv_to_bag(scan_csvfile, odom_csvfile, bagfile):
#     scan_df = pd.read_csv(scan_csvfile)
#     odom_df = pd.read_csv(odom_csvfile)
    
#     # 임시로 처음 1000개의 스캔만 변환
#     scan_df = scan_df.head(1000)
#     odom_df = odom_df.head(1000)

#     with rosbag.Bag(bagfile, 'w') as bag:
#         for index, row in scan_df.iterrows():
#             scan_msg = create_laserscan_message(row)
#             bag.write('/scan', scan_msg, t=scan_msg.header.stamp)

#             # Create and write the TF transform message for the laser
#             transform = create_transform_message(
#                 scan_msg.header.stamp,
#                 'base_footprint',  # Parent frame
#                 'laser',  # Child frame
#                 [0.0, 0.0, 0.0],  # Translation
#                 [0.0, 0.0, 0.0, 1.0]  # Rotation (identity quaternion)
#             )
#             tf_msg = TFMessage([transform])
#             bag.write('/tf', tf_msg, t=transform.header.stamp)

#         for index, row in odom_df.iterrows():
#             odom_msg = create_odometry_message(row)
#             bag.write('/odom', odom_msg, t=odom_msg.header.stamp)

#             # Create and write the TF transform message for the odom
#             transform = create_transform_message(
#                 odom_msg.header.stamp,
#                 'odom',  # Parent frame for odometry
#                 odom_msg.child_frame_id,  # Child frame from odometry message
#                 [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z],
#                 [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
#             )
#             tf_msg = TFMessage([transform])
#             bag.write('/tf', tf_msg, t=transform.header.stamp)

#     print(f'Bag file {bagfile} created.')

# if __name__ == '__main__':
#     rospy.init_node('csv_to_bag', anonymous=True)
#     csv_to_bag(scan_csv_path, odom_csv_path, bag_path)
