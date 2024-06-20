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

    x_coords = []
    y_coords = []
    xy_color = []
    filtered_x_coords = []
    filtered_y_coords = []
    filtered_xy_color = []
    noise_x_coords = []
    noise_y_coords = []
    noise_color = []

    for i in range(length):
        x = result_ranges[i] * np.cos(angles[i])
        y = result_ranges[i] * np.sin(angles[i])
        if labels[i] == -1:
            if result_ranges[i] == float('inf'):
                noise_x_coords.append(original_ranges[i] * np.cos(angles[i]))
                noise_y_coords.append(original_ranges[i] * np.sin(angles[i]))
                noise_color.append('red')
            else:
                noise_x_coords.append(x)
                noise_y_coords.append(y)
                noise_color.append('red')
        else:
            noise_x_coords.append(x)
            noise_y_coords.append(y)
            noise_color.append('black')
            x_coords.append(x)
            y_coords.append(y)
            xy_color.append(colors[labels[i] % len(colors)])
            filtered_x_coords.append(x)
            filtered_y_coords.append(y)
            filtered_xy_color.append(colors[labels[i] % len(colors)])

    # 왼쪽 subplot: 노이즈(red) 및 나머지(black) 데이터
    ax1.scatter(noise_x_coords, noise_y_coords, c=noise_color, s=7)
    ax1.scatter([0], [0], c='red', s=20)
    ax1.set_title('DBSCAN Result with Noise Highlighted')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')

    # 가운데 subplot: 전체 데이터 포함
    ax2.scatter(x_coords, y_coords, c=xy_color, s=7)
    ax2.scatter([0], [0], c='red', s=20)
    ax2.set_title(f'DBSCAN Result with Noise, labels = {len(set(labels))}')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')

    # 오른쪽 subplot: 노이즈 데이터 제외
    ax3.scatter(filtered_x_coords, filtered_y_coords, c=filtered_xy_color, s=7)
    ax3.scatter([0], [0], c='red', s=20)
    ax3.set_title(f'DBSCAN Result without Noise, labels = {len(set(filtered_xy_color))}')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')

    stamp = 0
    image_file_path = os.path.join(folder_path, f'lidar_data_{stamp}.png')
    while os.path.exists(image_file_path):
        stamp += 1
        image_file_path = os.path.join(folder_path, f'lidar_data_{stamp}.png')
    
    plt.savefig(image_file_path)
    plt.close()
