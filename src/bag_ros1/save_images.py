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
    fig, ax = plt.subplots(figsize=(10, 10))
    
    length = len(result_ranges)
    angles = [angle_min + i * angle_increment for i in range(length)]

    x_coords = []
    y_coords = []
    xy_color = []
    for i in range(length):
        if labels[i] == -1:
            if original_ranges[i] != float('inf'):
                x_coords.append(original_ranges[i] * np.cos(angles[i]))
                y_coords.append(original_ranges[i] * np.sin(angles[i]))
                xy_color.append('red')
        else:
            x_coords.append(result_ranges[i] * np.cos(angles[i]))
            y_coords.append(result_ranges[i] * np.sin(angles[i]))
            xy_color.append(colors[labels[i] % len(colors)])

    ax.scatter(x_coords, y_coords, c=xy_color, s=10, label='trained data')

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
