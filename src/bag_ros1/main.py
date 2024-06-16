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


def calculate(eps_ratios, strides, make_output, make_static, make_image):
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

            print(f"\n EPS={EPS_RATIO}, STRIDE={STRIDE} start")
            with rosbag.Bag(bag_path2, 'w') as outbag:
                for topic, msg, t in rosbag.Bag(bag_path).read_messages():
                    if topic == '/scan':
                        count += 1
                        if count % 2000 == 1:
                            print(f"Index {count-1} data completed.")
                            msg, time2, remove = compute_DBSCAN(msg, eps_ratio=EPS_RATIO, stride=STRIDE, make_image=make_image)
                        else:
                            msg, time2, remove = compute_DBSCAN(msg, eps_ratio=EPS_RATIO, stride=STRIDE, make_image=False)
                        calTime += time2
                        removed += remove
                    if make_output:
                        outbag.write(topic, msg, t)
            
            # Collect results
            avg_removed = removed / count if count != 0 else 0
            avg_calTime = calTime / count if count != 0 else 0
            
            results.append({
                'EPS_RATIO': EPS_RATIO,
                'STRIDE': STRIDE,
                'total_scan_msg': count,
                'total_removed': removed,
                'average_removed_data': avg_removed,
                'total_time': calTime,
                'time_used_for_DBSCAN': avg_calTime
            })

    if make_static:
        # Convert results to a DataFrame
        results_df = pd.DataFrame(results)

        # Save results to an Excel file
        excel_path = os.path.join(project_path, 'dbscan_results.xlsx')
        results_df.to_excel(excel_path, index=False)
        print(f"Results saved to {excel_path}")


if __name__ == '__main__':
    eps_ratios = [10, 30, 50, 70, 90]
    strides = [2, 4, 6, 8, 10]
    calculate(eps_ratios, strides, make_output=True, make_static=True, make_image=True)
