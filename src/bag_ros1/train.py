import os
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN


# preprocess data.
def preprocess(angle_min, angle_increment, ranges):
    angles = []
    distances = []

    for i in range(len(ranges)):
        if np.isfinite(ranges[i]):
            current_angle = angle_min + i * angle_increment
            angles.append(current_angle)
            distances.append(ranges[i])
        else:
            print("INF!!")
    
    return angles, distances

# create train data
def calculate_distance(angle1, angle2, distance1, distance2):
    theta_diff = np.arctan2(np.sin(angle1 - angle2), np.cos(angle1 - angle2))
    return np.sqrt(distance1**2 + distance2**2 - 2 * distance1 * distance2 * np.cos(theta_diff))

def make_train_data(angle, distance):
    assert len(angle) == len(distance)

    length = len(angle)
    rows = []

    prev_angle = angle[-1]
    prev_distance = distance[-1]
    
    prev_angle_diff = np.arctan2(np.sin(prev_angle - angle[0]), np.cos(prev_angle - angle[0]))
    prev_dist_diff = np.abs(prev_distance - distance[0])
    prev_inter_distance = calculate_distance(prev_angle, angle[0], prev_distance, distance[0])

    for i in range(length):
        c2 = i
        c3 = (i + 1) % length

        angleleft = prev_angle_diff
        distleft = prev_dist_diff
        interleft = prev_inter_distance

        angleright = np.arctan2(np.sin(angle[c2] - angle[c3]), np.cos(angle[c2] - angle[c3]))
        distright = np.abs(distance[c2] - distance[c3])
        interright = calculate_distance(angle[c2], angle[c3], distance[c2], distance[c3])

        rows.append({
            'AngleLeft': angleleft,
            'AngleRight': angleright,
            'DistLeft': distleft,
            'DistRight': distright,
            'InterLeft': interleft,
            'InterRight': interright
        })

        prev_angle_diff = angleright
        prev_dist_diff = distright
        prev_inter_distance = interright
    
    df = pd.DataFrame(rows)

    return df

def save_images(angles, distances, frame_ids, times, clusters, image_counter):
    fig, axs = plt.subplots(3, 3, figsize=(15, 15))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    axs = axs.ravel()

    for i in range(len(angles)):
        angle = angles[i]
        distance = distances[i]
        frame_id = frame_ids[i]
        time = times[i]
        cluster = clusters[i]

        # x, y 좌표 계산
        x_coords = [d * np.cos(a) for d, a in zip(distance, angle)]
        y_coords = [d * np.sin(a) for d, a in zip(distance, angle)]

        # 시각화
        unique_labels = set(cluster)

        for label in unique_labels:
            label_mask = (cluster == label)
            if label == -1:
                color = 'red'
            else:
                color = 'blue'
            axs[i].scatter(np.array(x_coords)[label_mask], np.array(y_coords)[label_mask], c=color, label=f'Cluster {label}')

        axs[i].set_title(f'Frame ID: {frame_id} Timestamp: {time}')
        axs[i].set_xlabel('X')
        axs[i].set_ylabel('Y')

    image_file_path = os.path.join(png_path, f'frames_{image_counter}.png')
    plt.savefig(image_file_path)
    plt.close()


def compute_DBSCAN(msg, eps_ratio, stride):
    start_time = time.time()

    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges, dtype=np.float32)

    Angle, Distance = preprocess(angle_min, angle_increment, ranges)
    trainDF = make_train_data(Angle, Distance)
    
    # DBSCAN 클러스터링
    eps_value = np.percentile(trainDF['InterLeft'], eps_ratio)
    dbscan = DBSCAN(eps=eps_value, min_samples=5)
    labels = dbscan.fit_predict(trainDF)

    # 클러스터 -1 제거
    noise_indices = np.where(labels == -1)[0]

    # ranges 배열에서 노이즈 데이터 인덱스를 float('inf')로 변환
    valid_mask = np.isfinite(ranges) & (ranges > 0)
    original_indices = np.where(valid_mask)[0]
    noise_original_indices = original_indices[noise_indices]
    ranges[noise_original_indices] = float('inf')

    # 변경된 ranges를 원본 msg에 반영(+stride)
    range2 = ranges.tolist()
    indices = [i for i in range(0, len(range2), stride)]
    msg.ranges = [range2[i] for i in indices]

    end_time = time.time() - start_time

    # 제거된 인덱스 수 반환
    num_removed_indices = len(noise_indices)

    return msg, end_time, num_removed_indices