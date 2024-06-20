import matplotlib.pyplot as plt
import numpy as np
import os

# color
colors = [
    'blue',    # 'b'
    'green',   # 'g'
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

# 이미지 저장
def save_images(folder_path, angle_min, angle_increment, original_ranges, result_ranges, labels):
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(30, 10))
    
    length = len(result_ranges)
    angles = [angle_min + i * angle_increment for i in range(length)]

    # 상위 클러스터
    x_top = []
    y_top = []
    color_top = []

    # 노이즈 데이터
    x_noise = []
    y_noise = []

    for i in range(length):
        x = result_ranges[i] * np.cos(angles[i])
        y = result_ranges[i] * np.sin(angles[i])
        if labels[i] != -1: # valid
            x_top.append(x)
            y_top.append(y)
            color_top.append(colors[labels[i] % len(colors)])
        elif labels[i] == -1: # noise
            x_noise.append(original_ranges[i] * np.cos(angles[i]))
            y_noise.append(original_ranges[i] * np.sin(angles[i]))

    # 1번째: 노이즈(red), 나머지(black)
    ax1.scatter(x_top, y_top, c='black', s=7)
    ax1.scatter(x_noise, y_noise, c='red', s=7)
    ax1.scatter([0], [0], c='black', s=30)
    ax1.set_title('Cluster Result\nconfirmed data(black), noise data(red)')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')

    # 2번째: 노이즈(red), 나머지(color)
    ax2.scatter(x_top, y_top, c=color_top, s=7)
    ax2.scatter(x_noise, y_noise, c='red', s=7)
    ax2.scatter([0], [0], c='black', s=30)
    ax2.set_title('Cluster Result\nconfirmed data(diff.), noise data(red)')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')

    # 3번째: 나머지(color)
    ax3.scatter(x_top, y_top, c=color_top, s=7)
    ax3.scatter([0], [0], c='black', s=30)
    ax3.set_title('Cluster Result\nonly confirmed data(diff.)')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')

    stamp = 0
    image_file_path = os.path.join(folder_path, f'lidar_data_{stamp}.png')
    while os.path.exists(image_file_path):
        stamp += 1
        image_file_path = os.path.join(folder_path, f'lidar_data_{stamp}.png')
    
    plt.savefig(image_file_path)
    plt.close()
