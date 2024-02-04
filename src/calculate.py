# 여러가지 계산 함수
from math import atan2, cos, pi, radians, sin, sqrt

import numpy as np


# 두 좌표의 거리 반환
def calculate_dist(cord1:tuple, cord2:tuple) -> float:
    return sqrt((cord1[0]-cord2[0])**2 + (cord1[1]-cord2[1])**2)

# 두 직선이 이루는 각을 계산
def calculate_angle(m1, m2):
    angle_rad = atan2((m2 - m1) , (1 + m1 * m2))
    angle_deg = angle_rad * 180 / pi

    return angle_deg

# 좌표를 이동, 회전
def calculate_move(x, y, movex, movey, rotate) -> tuple:
    x += movex
    y += movey
    theta = radians(rotate)
    x = cos(theta) * x - sin(theta) * y
    y = sin(theta) * x + cos(theta) * y
    return (x, y)

# 이상치 데이터 반환
def checkOutlier(data:list):
    mean = np.mean(np.array(data))
    std = np.std(np.array(data))

    newData = []
    for d in data:
        zScore = (d-mean)/std
        if abs(zScore) > 0.5:
            newData.append(d)
    
    return newData

# 이상치 제거 후 평균, 표준편차 계산
def removeOutlier(data:list):
    mean = np.mean(np.array(data))
    std = np.std(np.array(data))

    newData = []
    for d in data:
        zScore = (d-mean)/std
        if abs(zScore) <= 1:
            newData.append(d)
    
    newMean = np.mean(np.array(newData))
    newStd = np.std(np.array(newData))
    return newMean, newStd

# 데이터값의 확률분포 계산
def check95(mean, std):
    zScore = 1.96
    max = mean + zScore * std
    min = mean - zScore * std

    return min, max

# 데이터의 분산을 계산
def calculateVariance(data):
    length = len(data)
    mean = sum(data)/n
    return sum((x-mean)**2 for x in data) / length

# 두 좌표의 중점 반환
def midCord(c1:tuple, c2:tuple) -> tuple:
    return ((c1[0]+c2[0])/2, (c1[1]+c2[1])/2)

def checkFunc(funcDiff:list):
    return 0.9, 1.1
    

# 점 c1, c2 중 직선 밖에 있는 점을 계산하여 해당 점을 기준으로 새로운 시작/끝 좌표 생성
def newCord(func, c1, c2, cmid, funcmid):
    # 각 점에서 내린 수선의 발과 직선의 중점 간 거리를 계산
    footCord1 = get_perpendicular(c1, func, cmid)
    footCord2 = get_perpendicular(c2, func, cmid)

    footDist1 = sqrt((funcmid[0]-footCord1[0])**2 + (funcmid[1]-footCord1[1])**2)
    footDist2 = sqrt((funcmid[0]-footCord2[0])**2 + (funcmid[1]-footCord2[1])**2)

    # c1, c2중 거리가 큰 점의 수선의 발을 새로운 점으로 설정
    return footCord1 if footDist1 > footDist2 else footCord2


# 직선 y=ax+b를 구하고 점 (x,y)에서 내린 수선과의 교차점의 좌표를 계산
def get_perpendicular(cord1, func, funcCord):
    # 직선 y=ax+b의 값을 계산. a=func, 직선 위의 점 funcCord
    a = func
    b = funcCord[1] - funcCord[0] * func
    x = cord1[0]
    y = cord1[1]
    # 수선의 방정식: y = -1/a * x + c (c는 상수)
    c = y + 1/a * x

    # 교차점의 x, y 좌표 계산
    intersection_x = (b - c) / (1/a + a)
    intersection_y = -1/a * intersection_x + c
    return intersection_x, intersection_y

    # 수선의 길이 계산
    distance = sqrt((x - intersection_x)**2 + (y - intersection_y)**2)

    return distance

# 점(x,y)에서 직선에 내린 수선의 길이를 계산
