import os
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

# 데이터 전처리
def preprocess(angle_min, angle_increment, ranges, stride):
    ranges = np.nan_to_num(ranges, nan=float('inf'))
    angles = []

    current_angle = angle_min - angle_increment
    for _ in range(len(ranges)):
        current_angle += angle_increment
        angles.append(current_angle)

    distances2 = [value for index, value in enumerate(ranges) if index % stride == 0]
    angles2 = [value for index, value in enumerate(angles) if index % stride == 0]
    return angles2, distances2

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


def save_images(folder_path, angle_min, angle_increment, original_ranges, result_ranges, stride):
    fig, ax = plt.subplots(figsize=(10, 10))

    length = len(result_ranges)
    angles = [angle_min + i * angle_increment for i in range(length)]

    x_coords = [r * np.cos(a) for r, a in zip(original_ranges, angles)]
    y_coords = [r * np.sin(a) for r, a in zip(original_ranges, angles)]

    stride_x = [x_coords[j] for j in range(length) if j % stride == 0]
    stride_y = [y_coords[j] for j in range(length) if j % stride == 0]
    stride_colors = ['red' if result_ranges[j] == float('inf') 
                    else 'blue' for j in range(length) if j % stride == 0]
    
    non_stride_x = [x_coords[j] for j in range(length) if j % stride != 0]
    non_stride_y = [y_coords[j] for j in range(length) if j % stride != 0]
    
    ax.scatter(non_stride_x, non_stride_y, c='black', s=7, label='excluded data')
    ax.scatter(stride_x, stride_y, c=stride_colors, s=20, label='trained data')

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




def compute_DBSCAN(msg, eps_ratio, stride, make_image):
    start_time = time.time()

    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges, dtype=np.float32)
    length = len(ranges)

    Angle, Distance = preprocess(angle_min, angle_increment, ranges, stride)
    trainDF = make_train_data(Angle, Distance)

    # DBSCAN 클러스터링
    eps_value = np.percentile(trainDF['InterLeft'], eps_ratio)
    dbscan = DBSCAN(eps=eps_value, min_samples=5)
    labels = dbscan.fit_predict(trainDF)

    # 이상치 및 제거한 값들을 다시 추가
    removed_count = 0
    result_ranges = []
    label_index = 0

    for i in range(length):
        if i % stride == 0:
            if labels[label_index] != -1:
                result_ranges.append(ranges[i])
            else:
                result_ranges.append(float('inf'))
                removed_count += 1
            label_index += 1
        else:
            result_ranges.append(float('inf'))

    end_time = time.time() - start_time

    # 이미지 생성
    if make_image:
        img_folder_path = os.path.join(log_path, 'image', f'{eps_ratio}_{stride}')
        os.makedirs(img_folder_path, exist_ok=True)
        save_images(img_folder_path, angle_min, angle_increment, msg.ranges, result_ranges, stride)
    
    # 결과 반영
    msg.ranges = result_ranges

    return msg, end_time, removed_count
