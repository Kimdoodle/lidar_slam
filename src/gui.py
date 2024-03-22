import matplotlib.pyplot as plt
import math

'''
    data structure
    [[x1,y1], [x2,y2], [x3,y3], [x4,y4]...]: double list
'''

# odometry 표시
def graphics_odometry(data):
    x = [cord[0] for cord in data]
    y = [cord[1] for cord in data]

    plt.scatter(x, y, marker='o', color='red')

# 좌표 표시
def graphics_cord(data, color):
    cordX = []
    cordY = []
    for info in data:
        x = info[1]*math.cos(math.radians(info[0]))
        y = info[1]*math.sin(math.radians(info[0]))
        cordX.append(x)
        cordY.append(y)

    plt.scatter(cordX, cordY, marker='o', s=5, color=color)


def confirm():
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid()
    plt.show()
