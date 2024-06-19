import os
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

# color
colors = [
    'blue',    # 'b'
    'green',   # 'g'
    'red',     # 'r'
    'cyan',    # 'c'
    'magenta', # 'm'
    'yellow',  # 'y'
    'black',   # 'k'
    'orange',
    'purple',
    'brown',
    'pink',
    'gray',
    'olive',
    'navy',
    'white'    # 'w'
]

# 데이터 전처리
def preprocess(angle_min, angle_increment, ranges):
    ranges = np.nan_to_num(ranges, nan=float('inf'))
    angles = [angle_min + i * angle_increment for i in range(len(ranges))]

    return angles, ranges

# 두 점 간 거리 계산
def calculate_distance(angle1, angle2, distance1, distance2):
    theta_diff = np.arctan2(np.sin(angle1 - angle2), np.cos(angle1 - angle2))
    return np.sqrt(distance1**2 + distance2**2 - 2 * distance1 * distance2 * np.cos(theta_diff))

# 학습 데이터 생성
def make_train_data(angle, distance):
    assert len(angle) == len(distance)

    length = len(angle)
    rows = []

    prev_angle = angle[-1]
    prev_distance = distance[-1]
    
    # prev_angle_diff = np.arctan2(np.sin(prev_angle - angle[0]), np.cos(prev_angle - angle[0]))
    prev_dist_diff = np.abs(prev_distance - distance[0])
    prev_inter_distance = calculate_distance(prev_angle, angle[0], prev_distance, distance[0])

    for i in range(length):
        c2 = i
        c3 = (i + 1) % length

        # angleleft = prev_angle_diff
        distleft = prev_dist_diff
        interleft = prev_inter_distance

        # angleright = np.arctan2(np.sin(angle[c2] - angle[c3]), np.cos(angle[c2] - angle[c3]))
        distright = np.abs(distance[c2] - distance[c3])
        interright = calculate_distance(angle[c2], angle[c3], distance[c2], distance[c3])

        rows.append({
            # 'AngleLeft': angleleft,
            # 'AngleRight': angleright,
            'DistLeft': distleft,
            'DistRight': distright,
            'InterLeft': interleft,
            'InterRight': interright
        })

        # prev_angle_diff = angleright
        prev_dist_diff = distright
        prev_inter_distance = interright
    
    df = pd.DataFrame(rows)

    return df

# save images - various colors.
def save_images(folder_path, angle_min, angle_increment, original_ranges, result_ranges, labels):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    length = len(result_ranges)
    angles = [angle_min + i * angle_increment for i in range(length)]

    x_coords = [r * np.cos(a) for r, a in zip(original_ranges, angles)]
    y_coords = [r * np.sin(a) for r, a in zip(original_ranges, angles)]

    scatter_colors = []
    for label in labels:
        if label == -1:
            scatter_colors.append('red')
        else:
            scatter_colors.append(colors(label % 15))

    ax.scatter(x_coords, y_coords, c=scatter_colors, s=15, label='trained data')

    ax.set_title('DBSCAN Result')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()

    stamp = 0
    image_file_path = os.path.join(folder_path, f'lidar_data_{stamp}.png')
    while os.path.exists(image_file_path):
        stamp += 1
        image_file_path = os.path.join(folder_path, f'lidar_data_{stamp}.png')
    
    plt.savefig(image_file_path)
    plt.close()

    
def compute_Cluster(msg, eps_ratio, remains, make_image):
    start_time = time.time()

    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges, dtype=np.float32)
    length = len(ranges)

    Angle, Distance = preprocess(angle_min, angle_increment, ranges)
    trainDF = make_train_data(Angle, Distance)

    # make Clusters
    threshold = np.percentile(trainDF['InterLeft'], eps_ratio)
    labels = []
    counts = [0]
    current_label = 0

    for i in range(len(ranges) - 1):  # Adjust for the last element not having a next interleft
        if trainDF['InterLeft'][i] > threshold:
            current_label += 1
            counts.append(0)
        counts[-1] += 1
        labels.append(current_label)

    # Append the label for the last element
    labels.append(current_label)
    
    # Compare first and last clusters
    if trainDF['InterLeft'][0] <= threshold:
        # merge first and last cluster
        counts[0] += counts[-1]
        labels[-counts[-1]:] = [labels[0]] * counts[-1]
        counts = counts[:-1]
        labels = labels[:-counts[-1]]

    # Remove except top {remains} clusters
    if remains < len(counts):
        top_clusters = np.argsort(counts)[-remains:]
        top_labels = set(top_clusters)

        # Update ranges and labels
        for i, label in enumerate(labels):
            if label not in top_labels:
                labels[i] = -1
                ranges[i] = float('inf')

    end_time = time.time() - start_time

    # 이미지 생성
    if make_image:
        img_folder_path = os.path.join(log_path, 'image', f'{eps_ratio}_{threshold}')
        os.makedirs(img_folder_path, exist_ok=True)
        save_images(img_folder_path, angle_min, angle_increment, msg.ranges, labels)
    
    # 결과 반영
    msg.ranges = ranges

    return msg, end_time
