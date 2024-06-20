import os
import time

import numpy as np
from preprocess import make_train_data_DBSCAN2, preprocess
from save_images import save_images
from sklearn.cluster import DBSCAN
from sklearn.metrics import pairwise_distances
from sklearn.preprocessing import MinMaxScaler

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')


# DBSCAN을 이용한 클러스터링
def compute_DBSCAN(msg, eps_ratio, minpts, remains, make_image):
    start_time = time.time()

    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = np.array(msg.ranges, dtype=np.float32)

    Angle, Distance = preprocess(angle_min, angle_increment, ranges)
    assert len(Angle) == len(Distance)
    trainDF, removed_indices = make_train_data_DBSCAN2(Angle, Distance)

    # 데이터 정규화
    # scaler = MinMaxScaler()
    # trainDF = scaler.fit_transform(trainDF)

    # 클러스터 생성
    distances = pairwise_distances(trainDF)
    eps_value = np.percentile(distances, eps_ratio)
    dbscan = DBSCAN(eps=eps_value, min_samples=minpts)
    labels = dbscan.fit_predict(trainDF)

    # 상위 {remains} 비율의 클러스터만 남김
    unique_labels, counts = np.unique(labels, return_counts=True)
    sorted_indices = np.argsort(counts)[::-1]  # descending order
    total_clusters = len(unique_labels)
    clusters_to_keep = max(1, int(total_clusters * remains))
    top_clusters = unique_labels[sorted_indices[:clusters_to_keep]]
    top_labels = set(top_clusters)

    # 클러스터링 결과 반영
    for rm in removed_indices:
        labels = np.insert(labels, rm, -1)
    for i in range(len(labels)):
        if labels[i] != -1 and labels[i] not in top_labels:
            labels[i] = -1
    filtered_ranges = np.full_like(msg.ranges, float('inf'))
    for i in range(len(filtered_ranges)):
        if labels[i] != -1:
            filtered_ranges[i] = msg.ranges[i]

    end_time = time.time() - start_time

    # 이미지 생성
    if make_image:
        img_folder_path = os.path.join(log_path, 'image', f'{eps_ratio}_{remains}')
        os.makedirs(img_folder_path, exist_ok=True)
        save_images(img_folder_path, angle_min, angle_increment, msg.ranges, filtered_ranges, labels)

    # 결과 반영
    msg.ranges = filtered_ranges

    return msg, end_time, len(removed_indices)
