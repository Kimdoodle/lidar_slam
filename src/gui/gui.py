import matplotlib.pyplot as plt
import numpy as np
import time

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
def makeList(data) -> tuple:
    cordX = []
    cordY = []
    for info in data:
        x = info[1] * np.cos(np.radians(info[0]))
        y = info[1] * np.sin(np.radians(info[0]))
        cordX.append(x)
        cordY.append(y)
    return cordX, cordY

def graphics_cord(data, pausetime):
    confirm()
    infos = [makeList(info) for info in data]
    print("info 분석 완료")
    colors = ['black', 'red', 'blue', 'green', 'brown']
    for i in range(len(data)-4):    
        plt.clf()
        lists = infos[i:i+5]
        for j in range(5):
            plt.scatter(lists[j][0], lists[j][1], marker='o', s=5, color=colors[j])
    
        plt.pause(pausetime)


def confirm():
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid()
    plt.ion()
    plt.show()
