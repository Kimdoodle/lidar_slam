# 스캔 데이터를 비교하고 odometry를 추정
import copy
from math import cos, sqrt

import numpy as np
from scanLog import loadScanLog
from scipy.spatial import cKDTree
from unit import Cord


# 두 스캔 데이터의 유사성을 비교하는 함수
# input: list[Cord]
# 가장 유사성을 띄는 이동을 체크
def compare(cord1, cord2) -> tuple:
    # # 각도 유사성 체크
    # checkList = [0, 90, 180, 270]
    # checked = []
    # for angle in checkList:
    #     temp = rotateCord(cord2, angle)
    #     checked.append(check(cord1, temp))
    
    # index1 = checked.index(min(checked))
    # checked.pop(index1)
    # index2 = checked.index(min(checked))
    # min_value = checkList[index2]
    # max_value = checkList[index1]

    # while round(min_value, 2) < round(max_value, 2):
    #     avg = (min_value + max_value) / 2
    #     c1 = check(cord1, rotateCord(cord2, avg))
    #     c2 = check(cord1, rotateCord(cord2, max_value))
    #     if c1 < c2:
    #         max_value = avg
    #     else:
    #         min_value = avg
    #     print(f'min: {min_value}, max:{max_value}')
    
    # opt_angle = round((min_value + max_value)/2, 2)
    # print(f'opt_angle: {opt_angle}')

    # cord2 = rotateCord(cord2, opt_angle)

    # 이동 유사성 체크
    # x
    crit = 100
    min_value = 0
    max_value = crit
    while True:
        before = check(cord1, moveCord(cord2, min_value, 0))
        after = check(cord1, moveCord(cord2, max_value, 0))
        mid = check(cord1, moveCord(cord2, crit/2, 0))
        
        if before > after:
            min_value += crit/2
            if mid > after:
                crit /= 2
            else:
                max_value += crit
        else:
            max_value -= crit/2
            if mid > before:
                crit /= 2
            else:
                before -= crit
        
        # 종료조건
        if max_value-min_value <= crit: break

    opt_x = round((min_value + max_value)/2, 2)
    print(f'opt_x: {opt_x}')

    cord2 = moveCord(cord2, opt_x, 0)

    # y
    min_value = 0
    max_value = crit
    while True:
        before = check(cord1, moveCord(cord2, 0, min_value))
        after = check(cord1, moveCord(cord2, 0, max_value))
        mid = check(cord1, moveCord(cord2, 0, crit/2))
        
        if before > after:
            min_value += crit/2
            if mid > after:
                crit /= 2
            else:
                max_value += crit
        else:
            max_value -= crit/2
            if mid > before:
                crit /= 2
            else:
                before -= crit
        
        # 종료조건
        if max_value-min_value <= crit: break

    opt_y = round((min_value + max_value)/2, 2)
    print(f'opt_y: {opt_y}')

    cord2 = moveCord(cord2, 0, opt_y)

    print(f'test result: {check(cord1, cord2)}')
    return (opt_x, opt_y)

# # 모든 좌표를 특정 각도만큼 회전
# # input/output:list[Cord]
# def rotateCord(cordList, angle):
#     result = copy.deepcopy(cordList)
#     for cord in result:
#         cord.move_angle(angle)
#     return result

# 모든 좌표를 특정 거리만큼 이동
# input/output:list[Cord]
def moveCord(cordList, x, y):
    result = copy.deepcopy(cordList)
    for cord in result:
        cord.move_xy(x, y)
    return result

# 모든 좌표를 특정 각도, 거리만큼 이동 - Cord list
def move(cord, x, y):
    cord.move_xy(x, y)
    #cord.move_angle(angle)

# 두 좌표의 거리 유사성을 검사 - Nearest Neighbor Method
def check(cord1, cord2):
    before = np.array([(cord.x, cord.y) for cord in cord1])
    after = np.array([(cord.x, cord.y) for cord in cord2])
    beforeTree = cKDTree(before)
    distances, indices = beforeTree.query(after)
    mean = np.mean(distances)

    return mean


# 각 스캔 데이터의 특징을 추출하는 함수
# 각 점의 거리 확률분포를 계산하여 값이 큰 요소를 특징으로 설정.
def feature(cordInfo):
    interInfo = []
    length = len(cordInfo)
    for i in range(length):
        i2 = (i+1)%length
        d1 = cordInfo[i].distance
        d2 = cordInfo[i2].distance
        angle = cordInfo[i2].angle - cordInfo[i].angle

        distance = sqrt(d1**2 + d2**2 - (2*d1*d2*cos(angle)))
        interInfo.append(distance)
    
    mean = np.mean(np.array(interInfo))
    std = np.std(np.array(interInfo))
    zScore = 1.96
    max = mean + zScore * std
    min = mean - zScore * std

    # 거리가 큰 지점 간 점의 개수를 저장
    feat = []
    count = 0
    for i in range(length):
        if min <= interInfo[i] <= max:
            count += 1
            # 첫요소/마지막요소 결합여부 검사
            if i == length-1:
                feat[0] += count
        else:
            feat.append(count)
            count = 0

    # debug
    print(f'interInfo: \n{interInfo}')
    print(f'feat: \n{feat}')
    
    return feat

if __name__ == '__main__':
    logData = loadScanLog("./catkin_ws/src/rplidar_py/log2")
    data1 = [Cord(element) for element in logData[0]]
    data2 = [Cord(element) for element in logData[1]]
    print(compare(data1, data2))
