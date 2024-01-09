import time
from math import cos, radians, sin

import matplotlib.pyplot as plt
import numpy as np
import scanLog


# 좌표
class Cord:
    def __init__(self, data):
        self.quality = data[0]
        self.angle = data[1]
        self.distance = data[2]
        self.x = self.distance * np.cos(np.radians(self.angle))
        self.y = self.distance * np.sin(np.radians(self.angle))
    
    def toString(self):
        return(f'{self.quality}, {self.angle}, {self.distance}, {self.x}, {self.y}')

# 선
class Line:
    def __init__(self, cordI, cordII, indexI, indexII):
        self.sindex = indexI
        self.eindex = indexII
        self.startX = cordI.x
        self.startY = cordI.y
        self.endX = cordII.x
        self.endY = cordII.y
    
    def toString(self):
        return(f'{self.startX}, {self.startY}, {self.endX}, {self.endY}')

# 스캔　데이터
class Scan:
    def __init__(self, data):
        # 초기 데이터 삽입
        self.cordInfo = []  # 각 점의 좌표정보
        self.angleInfo = [] # 각 점 사이의 각도차이
        self.distInfo = []  # 중심에서 각 점까지의 거리 차이
        self.interInfo = [] # 각 점 사이의 거리차이
        self.funcInfo = []  # 두 점을 연결한 직선의 기울기
        self.lineInfo = []  # 완성된 직선 정보
        self.calculate_data(data)
        self.makeLine()

    # 초기 지도 데이터 분석
    def calculate_data(self, data):
        # 좌표정보 수집
        self.cordInfo = [Cord(element) for element in data]
        print(f'스캔 좌표 개수: {len(self.cordInfo)}')

        # 인접한 점 사이의 차이 정보 계산
        self.length = len(self.cordInfo)
        for i in range(self.length):
            cordI = self.cordInfo[i]
            cordII = self.cordInfo[(i+1)%self.length]
            
            # 각차이
            angleDiff = cordII.angle - cordI.angle
            if angleDiff < 0: angleDiff += 360
            self.angleInfo.append(angleDiff)

            # 중심에서의 거리차이
            distDiff = np.abs(cordII.distance - cordI.distance)
            self.distInfo.append(distDiff)
            
            # 점 사이의 거리, 기울기 차이
            distX = cordII.x - cordI.x
            distY = cordII.y - cordI.y
            self.interInfo.append(np.sqrt(distX**2 + distY**2))
            self.funcInfo.append(distY / distX)

        # 이상치 제거 확률분포 계산
        self.angleMean, self.angleStd = self.removeOutlier(self.angleInfo)
        self.distMean, self.distStd = self.removeOutlier(self.distInfo)
        self.interMean, self.interStd = self.removeOutlier(self.interInfo)
        self.funcMean, self.funcStd = self.removeOutlier(self.funcInfo)


        # debug - 계산결과 로그 저장
        #scanLog.saveCalLog(self)

    # 선 정보 계산
    def makeLine(self):
        '''
            계산한 정보에서 이상치를 제거 후 확률분포를 구함.
            1. 두 점을 연결한 직선의 기울기 차이
            2. 두 점의 간격
            3. 두 점의 각도차
            4. 두 점까지의 거리차
        '''

        # 각 데이터의 95% 신뢰구간 기준값을 계산
        angleMin, angleMax = self.check95(self.angleMean, self.angleStd)
        distMin, distMax = self.check95(self.distMean, self.distStd)
        interMin, interMax = self.check95(self.interMean, self.interStd)
        funcMin, funcMax = self.check95(self.funcMean, self.funcStd)

        i = 0
        start = self.cordInfo[i]
        startIndex = i
        while True:
            end = self.cordInfo[i]
            endIndex = i
            i2 = (i + 1) % self.length
            comp = self.cordInfo[i2]

            
            check = [
                (funcMin <= self.funcInfo[i] <= funcMax),
                (interMin <= self.interInfo[i] <= interMax),
                (angleMin <= self.angleInfo[i] <= angleMax),
                (distMin <= self.distInfo[i] <= distMax),
            ]

            if False in check:
                self.lineInfo.append(Line(start, end, startIndex, endIndex))
                start = comp

            if i2 == 0: break
            i = i2

    # 이상치 제거 후 평균, 표준편차 계산
    def removeOutlier(self, data:list):
        mean = np.mean(np.array(data))
        std = np.std(np.array(data))

        newData = []
        for d in data:
            zScore = (d-mean)/std
            if np.abs(zScore) <= 1:
                newData.append(d)
        
        newMean = np.mean(np.array(newData))
        newStd = np.std(np.array(newData))
        return newMean, newStd

    # 데이터값의 확률분포 계산
    def check95(self, mean, std):
        zScore = 1.96
        max = mean + zScore * std
        min = mean - zScore * std

        return min, max