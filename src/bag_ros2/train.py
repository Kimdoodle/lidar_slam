import os
from decimal import Decimal, getcontext

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN

getcontext().prec = 50

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

png_path = os.path.join(log_path, 'png')
csvfile = os.path.join(log_path, 'csvfile', '_scan.csv')

# Ensure directories exist
os.makedirs(png_path, exist_ok=True)

# preprocess data.
def preprocess(Angle_Min, Angle_Increment, Distance, Intensity):
    Angle_Min = Decimal(Angle_Min)
    Angle_Increment = Decimal(Angle_Increment)
    angle = []
    dist = []
    inte = []
    removed = []
    for i in range(len(Distance)):
        if (Distance[i] != 'inf') & (Intensity[i] != '0.0'):
            current_angle = Angle_Min + Decimal(i) * Angle_Increment
            angle.append(float(current_angle))
            dist.append(float(Decimal(Distance[i])))
            inte.append(float(Decimal(Intensity[i])))
        else:
            removed.append(i)
    return angle, dist, inte, removed

# create train data
def calculate_distance(angle1, angle2, distance1, distance2):
    theta_diff = np.arctan2(np.sin(angle1 - angle2), np.cos(angle1 - angle2))
    return np.sqrt(distance1**2 + distance2**2 - 2 * distance1 * distance2 * np.cos(theta_diff))

def make_train_data(angle, distance, intensity):
    assert len(angle) == len(distance) == len(intensity)

    length = len(angle)
    rows = []

    prev_angle = angle[-1]
    prev_distance = distance[-1]
    prev_intensity = intensity[-1]
    
    prev_angle_diff = np.arctan2(np.sin(prev_angle - angle[0]), np.cos(prev_angle - angle[0]))
    prev_dist_diff = np.abs(prev_distance - distance[0])
    prev_inter_distance = calculate_distance(prev_angle, angle[0], prev_distance, distance[0])

    for i in range(length):
        c1 = (i - 1) % length
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

if __name__ == '__main__':
    print("hello")
    df = pd.read_csv(csvfile)

    for index, row in df.iterrows():
        Timestamp = row['Timestamp']
        Frame_ID = row['Frame_ID']
        Angle_Min = row['Angle_Min']
        Angle_Max = row['Angle_Max']
        Angle_Increment = row['Angle_Increment']
        Time_Increment = row['Time_Increment']
        Scan_Time = row['Scan_Time']
        Range_Min = row['Range_Min']
        Range_Max = row['Range_Max']
        Distance = row['Distance'].split(';')
        Intensity = row['Intensity'].split(';')

        Angle, Distance, Intensity, removed = preprocess(Angle_Min, Angle_Increment, Distance, Intensity)

        train_data = make_train_data(Angle, Distance, Intensity)
        eps_value = np.percentile(train_data['InterLeft'], 95)
        
        # DBSCAN 클러스터링
        dbscan = DBSCAN(eps=eps_value, min_samples=5)
        train_data['Cluster'] = dbscan.fit_predict(train_data)


        # x, y 좌표 계산
        x_coords = [d * np.cos(a) for d, a in zip(Distance, Angle)]
        y_coords = [d * np.sin(a) for d, a in zip(Distance, Angle)]

        # 시각화
        plt.figure(figsize=(10, 10))
        unique_labels = set(train_data['Cluster'])

        for label in unique_labels:
            label_mask = (train_data['Cluster'] == label)
            if label == -1:
                color = 'red'  # 노이즈
            else:
                color = 'blue'  # 다른 클러스터
            plt.scatter(np.array(x_coords)[label_mask], np.array(y_coords)[label_mask], c=[color], label=f'Cluster {label}')

        plt.title(f'Frame ID: {Frame_ID} Timestamp: {Timestamp}')
        plt.xlabel('X')
        plt.ylabel('Y')

        image_file_path = os.path.join(png_path, f'{Frame_ID}_{Timestamp}.png')
        plt.savefig(image_file_path)
        plt.close()