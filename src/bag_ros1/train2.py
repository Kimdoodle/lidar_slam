import os
import time

import numpy as np
from save_images import save_images
from preprocess import preprocess, make_train_data

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')


# 단순 클러스터링
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

    # 상위 {remains} 비율의 클러스터만 남김
    total_clusters = len(counts)
    clusters_to_keep = max(1, int(total_clusters * remains))
    top_clusters = np.argsort(counts)[-clusters_to_keep:]
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
