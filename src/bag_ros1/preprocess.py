import numpy as np
import pandas as pd


# 데이터 전처리
def preprocess(angle_min, angle_increment, ranges):
    # ranges = np.nan_to_num(ranges, nan=float('inf'))
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


def make_train_data_DBSCAN(angle, distance):
    assert len(angle) == len(distance)

    length = len(angle)
    rows = []

    # float('inf')인 데이터는 따로 보관
    valid_indices = [i for i in range(length) if distance[i] != float('inf')]
    removed_indices = [i for i in range(length) if i not in valid_indices]
    angle = [angle[i] for i in valid_indices]
    distance = [distance[i] for i in valid_indices]

    prev_angle = angle[-1]
    prev_distance = distance[-1]
    
    prev_angle_diff = np.arctan2(np.sin(prev_angle - angle[0]), np.cos(prev_angle - angle[0]))
    prev_dist_diff = np.abs(prev_distance - distance[0])
    prev_inter_distance = calculate_distance(prev_angle, angle[0], prev_distance, distance[0])

    for i in range(len(valid_indices)):
        angleleft = prev_angle_diff
        distleft = prev_dist_diff
        interleft = prev_inter_distance

        rows.append({
            'AngleInfo': angleleft,
            'DistInfo': distleft,
            'InterInfo': interleft
        })

        prev_angle_diff = np.arctan2(np.sin(angle[i] - angle[(i + 1) % len(valid_indices)]), np.cos(angle[i] - angle[(i + 1) % len(valid_indices)]))
        prev_dist_diff = np.abs(distance[i] - distance[(i + 1) % len(valid_indices)])
        prev_inter_distance = calculate_distance(angle[i], angle[(i + 1) % len(valid_indices)], distance[i], distance[(i + 1) % len(valid_indices)])
    
    df = pd.DataFrame(rows)

    return df, removed_indices


# x, y좌표를 훈련 데이터로 삽입
def make_train_data_DBSCAN2(angle, distance):
    assert len(angle) == len(distance)

    length = len(angle)
    rows = []

    # float('inf')인 데이터는 따로 보관
    valid_indices = [i for i in range(length) if distance[i] != float('inf')]
    removed_indices = [i for i in range(length) if i not in valid_indices]
    angle = [angle[i] for i in valid_indices]
    distance = [distance[i] for i in valid_indices]

    # x, y 좌표 계산
    x_coords = [distance[i] * np.cos(angle[i]) for i in range(len(angle))]
    y_coords = [distance[i] * np.sin(angle[i]) for i in range(len(angle))]

    for i in range(len(valid_indices)):
        x = x_coords[i]
        y = y_coords[i]
        # if i == len(valid_indices) - 1:
        #     nextIndex = 0
        # else:
        #     nextIndex = i + 1
        # next_x = x_coords[nextIndex]
        # next_y = y_coords[nextIndex]

        # inter_distance = calculate_distance(x, y, next_x, next_y)
        # dist_difference = np.abs(distance[i] - distance[nextIndex])
        rows.append({
            'X': x,
            'Y': y,
            # 'InterInfo': inter_distance,
            # 'DistInfo': dist_difference
        })

    df = pd.DataFrame(rows)

    return df, removed_indices