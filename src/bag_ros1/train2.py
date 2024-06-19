import os
import time

from save_images import save_images
import numpy as np
import pandas as pd

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')


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
    interleft = []
    removed = []

    # float('inf')인 데이터는 따로 보관
    valid_indices = [i for i in range(length) if distance[i] != float('inf')]
    angle = [angle[i] for i in valid_indices]
    distance = [distance[i] for i in valid_indices]

    for i in range(length):
        current = i
        before = (i-1) % length
        interleft.append(calculate_distance(angle[current], angle[before],
                                            distance[current], distance[before]))

    removed = [i for i in range(len(distance)) if distance[i] == float('inf')]

    return interleft, removed


# 클러스터 계산
def compute_Cluster(msg, eps_ratio, remains, make_image):
    start_time = time.time()

    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges, dtype=np.float32)

    Angle, Distance = preprocess(angle_min, angle_increment, ranges)
    assert len(Angle) == len(Distance)
    trainDF, removed = make_train_data(Angle, Distance)

    # 클러스터 생성
    valid_trainDF = np.array(trainDF)
    threshold = np.percentile(valid_trainDF, eps_ratio)
    labels = np.full(len(ranges), -1, dtype=int)
    counts = [0]
    current_label = 0

    valid_indices = [i for i in range(len(ranges)) if i not in removed]

    for i in range(len(valid_indices) - 1):
        if trainDF[i] > threshold:
            current_label += 1
            counts.append(0)
        counts[-1] += 1
        labels[valid_indices[i]] = current_label
    labels[valid_indices[-1]] = current_label

    # 처음/마지막 클러스터 비교 결합
    if trainDF[0] <= threshold:
        counts[0] += counts[-1]
        labels[valid_indices[-counts[-1]:]] = labels[valid_indices[0]]
        counts = counts[:-1]

    # 상위 {remains}개의 클러스터만 남김
    if remains < len(counts):
        top_clusters = np.argsort(counts)[-remains:]
        top_labels = set(top_clusters)

        # 결과 반영
        for i in valid_indices:
            if labels[i] not in top_labels:
                labels[i] = -1

    # label이 -1인 데이터의 값을 float('inf')로 처리
    for i in range(len(labels)):
        if labels[i] == -1:
            ranges[i] = float('inf')

    end_time = time.time() - start_time

    # 이미지 생성
    if make_image:
        img_folder_path = os.path.join(log_path, 'image', f'{eps_ratio}_{remains}')
        os.makedirs(img_folder_path, exist_ok=True)
        save_images(img_folder_path, angle_min, angle_increment, msg.ranges, ranges, labels)

    msg.ranges = ranges

    return msg, end_time, len(removed)
