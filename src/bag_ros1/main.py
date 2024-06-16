import os
import time

import pandas as pd
import rosbag
from train import compute_DBSCAN

# Define paths
file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

bagname = 'Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag'
bag_path = os.path.join(log_path, 'bag', bagname)

# Parameter ranges
eps_ratios = [80, 85, 90, 95, 98]
strides = list(range(1, 11))

# Results list
results = []

# Iterate over parameter combinations
for EPS_RATIO in eps_ratios:
    for STRIDE in strides:
        bagname2 = f'compressed_{EPS_RATIO}_{STRIDE}.bag'
        bag_path2 = os.path.join(log_path, 'bag', bagname2)
        
        calTime = 0.
        removed = 0
        count = 0

        with rosbag.Bag(bag_path2, 'w') as outbag:
            for topic, msg, t in rosbag.Bag(bag_path).read_messages():
                if topic == '/scan':
                    msg, time2, remove = compute_DBSCAN(msg, eps_ratio=EPS_RATIO, stride=STRIDE)
                    calTime += time2
                    removed += remove
                    count += 1
                    if count % 1000 == 1:
                        print(f"{count-1} index completed.")
                outbag.write(topic, msg, t)
        
        # Collect results
        avg_removed = removed / count if count != 0 else 0
        avg_calTime = calTime / count if count != 0 else 0
        
        results.append({
            'EPS_RATIO': EPS_RATIO,
            'STRIDE': STRIDE,
            'total_scan_msg': count,
            'average_removed_data': avg_removed,
            'time_used_for_DBSCAN': avg_calTime
        })

# Convert results to a DataFrame
results_df = pd.DataFrame(results)

# Save results to an Excel file
excel_path = os.path.join(project_path, 'dbscan_results.xlsx')
results_df.to_excel(excel_path, index=False)
print(f"Results saved to {excel_path}")
