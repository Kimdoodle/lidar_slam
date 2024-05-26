import datetime
import os
import sys
import threading
import traceback

import numpy as np
import pandas as pd
import scandata

shutdown_flag = threading.Event()

# Create folders for logging
def make_folders() -> tuple:
    folder_count = 0
    current_path = os.path.dirname(__file__)
    project_path = os.path.abspath(os.path.join(current_path, '..', '..'))
    cur_time = datetime.datetime.now().strftime("%Y-%m-%d") + f"({folder_count})"
    log_dir = os.path.join(project_path, 'log', cur_time)
    while os.path.exists(log_dir):
        folder_count += 1
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d") + f"({folder_count})"
        log_dir = os.path.join(project_path, 'log', cur_time)

    raw_path = os.path.join(log_dir, 'raw')
    conv_path = os.path.join(log_dir, 'convert')
    os.makedirs(raw_path)
    os.makedirs(conv_path)

    return raw_path, conv_path

# Save scan data log
def save_scan_log(scan: list, raw_path: str):
    timestamp = 0
    current_time = ''
    scan = np.array(scan)
    scan_data = np.array(scan[:, 1:])
    next_time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    if current_time != next_time:
        timestamp = 0
        current_time = next_time
    else:
        timestamp += 1
    filename = f'log_({timestamp}).csv'
    file_directory = os.path.join(raw_path, filename)

    # Convert to DataFrame
    df = pd.DataFrame(scan_data, columns=['angle', 'distance'])

    # Save to CSV file
    df.to_csv(file_directory, index=False)

    print(f"Scan data log successfully saved to {filename}, data: {len(scan_data)}")


# Convert scan data to training data
def make_train_data(raw_path: str, conv_path: str):
    count = 0
    file_list = sorted(os.listdir(raw_path))
    for file in file_list:
        temp = []
        df = pd.read_csv(os.path.join(raw_path, file))
        for _, row in df.iterrows():
            temp.append((row['angle'], row['distance']))

        scan = scandata.Scan(temp)
        scan.postProcess()

        df = pd.DataFrame({
            'angle': scan.scanInfo[:, 1],
            'distance': scan.scanInfo[:, 2],
            'x': scan.x,
            'y': scan.y,
            'InterInfoLeft': scan.interInfoLeft,
            'InterInfoRight': scan.interInfoRight,
            'AngleInfoLeft': scan.angleInfoLeft,
            'AngleInfoRight': scan.angleInfoRight,
            'DistInfoLeft': scan.distInfoLeft,
            'DistInfoRight': scan.distInfoRight,
            'class': 0
        })

        # Set output file path
        base_filename = os.path.splitext(file)[0]
        output_path = os.path.join(conv_path, f"{base_filename}_train.csv")

        # Save to CSV file
        df.to_csv(output_path, index=False)
        print(f"Train data saved to {output_path}, data: {len(df)}")
        count += 1

    print(f"{count} train data files saved.")

if __name__ == '__main__':
    raw_data_path = os.path.join(os.getcwd(), 'log', '2024-05-26(0)', 'raw')
    conv_data_path = os.path.join(os.getcwd(), 'log', '2024-05-26(0)', 'convert')
    make_train_data(raw_data_path, conv_data_path)