import matplotlib.pyplot as plt
import numpy as np

'''
    data structure
    [(x1, y1), (x2, y2), (x3, y3), ...]: list of tuples
'''

# odometry 표시
def graphics_odometry(data):
    x = [cord[0] for cord in data]
    y = [cord[1] for cord in data]

    plt.scatter(x, y, marker='o', color='red')
    confirm()

# 데이터 표시
def graphics_cord(data, pausetime):
    colors = ['black', 'red', 'blue', 'green', 'brown']
    num_colors = len(colors)
    plt.clf()
    data = np.array(data)

    plt.scatter(data[:, 0], data[:, 1], marker='o', s=5, color=colors[0 % num_colors])

    plt.pause(pausetime)

def confirm():
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid()
    plt.ion()
    plt.show()
