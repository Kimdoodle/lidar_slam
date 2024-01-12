import bisect
import time
from math import acos, cos, degrees, radians, sin, sqrt

import matplotlib.pyplot as plt
import numpy as np
import scanLog
from compare import compare, move
from icp import icp
from unit import Cord, Line, rotate_cord


# 스캔　데이터
class Scan:
    def __init__(self, data, index):
        self.cordInfo = [Cord(element) for element in data]  # 각 점의 좌표정보
        self.cordXY = [(cord.x, cord.y) for cord in self.cordInfo]
        self.interInfo = [] # 각 점 사이의 거리차이
    
# class Scan:
#     # 초기 데이터 삽입
#     def __init__(self, data, index):
#         self.index = index
#         self.cordInfo = [Cord(element) for element in data]  # 각 점의 좌표정보
#         self.angleInfo = [] # 각 점 사이의 각도차이
#         self.distInfo = []  # 중심에서 각 점까지의 거리 차이
#         self.interInfo = [] # 각 점 사이의 거리차이
#         self.funcInfo = []  # 두 점을 연결한 직선의 기울기행
#         self.lineInfo = []  # 완성된 직선 정보

#         # self.calculate_data(data)
#         # self.makeLine()
#         # self.lineLog = self.makeLog(self.lineInfo)   # 완성된 직선 정보의 로그

#     # 기존 데이터에서 업데이트
#     def add(self, data):
#         cordInfo = [Cord(element) for element in data]
#         # temp = self.cordInfo + cordInfo
#         # self.cordInfo = temp
#         #print(f'index: {self.index}, size:{len(self.cordInfo)}, {len(temp)}')
#         moveInfo = compare(self.cordInfo, cordInfo)
#         result = [move(cord, moveInfo) for cord in cordInfo]

#         self.cordInfo += result


#     # 초기 지도 데이터 분석
#     def calculate(self):
#         # debug
#         print(f"총 스캔 데이터 개수: {len(self.cordInfo)}")
        
#         # 인접한 점 사이의 차이 정보 계산
#         self.length = len(self.cordInfo)
#         for i in range(self.length):
#             cordI = self.cordInfo[i]
#             cordII = self.cordInfo[(i+1)%self.length]
            
#             # 각차이
#             angleDiff = cordII.angle - cordI.angle
#             if angleDiff < 0: angleDiff += 360
#             self.angleInfo.append(angleDiff)

#             # 중심에서의 거리차이
#             distDiff = np.abs(cordII.distance - cordI.distance)
#             self.distInfo.append(distDiff)
            
#             # 점 사이의 거리, 기울기 차이
#             distX = cordII.x - cordI.x
#             distY = cordII.y - cordI.y
#             self.interInfo.append(np.sqrt(distX**2 + distY**2))
#             self.funcInfo.append(distY / distX)

#         # 이상치 제거 확률분포 계산
#         self.angleMean, self.angleStd = self.removeOutlier(self.angleInfo)
#         self.distMean, self.distStd = self.removeOutlier(self.distInfo)
#         self.interMean, self.interStd = self.removeOutlier(self.interInfo)
#         self.funcMean, self.funcStd = self.removeOutlier(self.funcInfo)


#         # debug - 계산결과 로그 저장
#         #scanLog.saveCalLog(self)

#     # 선 정보 계산
#     def makeLine(self):
#         '''
#             계산한 정보에서 이상치를 제거 후 확률분포를 구함.
#             1. 두 점을 연결한 직선의 기울기 차이
#             2. 두 점의 간격
#             3. 두 점의 각도차
#             4. 두 점까지의 거리차
#         '''

#         # 각 데이터의 95% 신뢰구간 기준값을 계산
#         angleMin, angleMax = self.check95(self.angleMean, self.angleStd)
#         distMin, distMax = self.check95(self.distMean, self.distStd)
#         interMin, interMax = self.check95(self.interMean, self.interStd)
#         funcMin, funcMax = self.check95(self.funcMean, self.funcStd)

#         i = 0
#         start = self.cordInfo[i]
#         startIndex = i
#         while True:
#             end = self.cordInfo[i]
#             endIndex = i
#             i2 = (i + 1) % self.length
#             comp = self.cordInfo[i2]

            
#             check = [
#                 (funcMin <= self.funcInfo[i] <= funcMax),
#                 (interMin <= self.interInfo[i] <= interMax),
#                 (angleMin <= self.angleInfo[i] <= angleMax),
#                 (distMin <= self.distInfo[i] <= distMax),
#             ]

#             if False in check:
#                 self.lineInfo.append(Line(start, end, startIndex, endIndex))
#                 start = comp

#             if i2 == 0: break
#             i = i2
    
#     # 스캔 데이터 단순화 - 직선 수를 줄임
#     '''
#         현재 테스트중인 항목
#         중심점과 직선이 이루는 각도를 계산
#         인접한 각도차를 비교하여 비슷하다면 직선을 합침
#     '''
#     def simplify(self):
#         log = self.lineLog
#         mean = np.mean(np.array(log))
#         std = np.std(np.array(log))
#         max = mean + 1.96 * std
#         min = mean - 1.96 * std
#         print(f'직선 정보의 평균: {mean}, 표준편차: {std}')

#         ratioList = []
#         index = 0
#         angle = log[index]
#         while True:
#             if angle == 0: angle = 0.01
#             index = (index+1)%len(log)
#             angle2 = log[index]
#             ratioList.append(angle2/angle)
#             angle = angle2
#             if index == 0:
#                 break

#         mean, std = self.removeOutlier(ratioList)
#         max = mean + 0.5 * std
#         min = mean - 0.5 * std
#         print(f'나눈 정보의 평균: {mean}, 표준편차: {std}')
#         print(f'범위: {min} to {max}')
#         #print(temp)
        
#         try:
#             newLine_List = []
#             for index in range(len(self.lineInfo)):
#                 if min <= ratioList[index] <= max:
#                     i2 = (index+1)%len(self.lineInfo)
#                     l1 = self.lineInfo[index]
#                     l2 = self.lineInfo[i2]
#                     newLine = Line(l1.cordI, l2.cordII, index, i2)
#                     newLine_List.append(newLine)
#             self.simplifiedLineInfo = newLine_List
#         except Exception as e:
#             print(e)


#     # 스캔 파일 로그화 - 각 직선과 직선중점 - 중심의 각도를 계산
#     def makeLog(self, lineInfo:list):
#         logList = []
#         try:
#             for line in lineInfo:
#                 start = self.cordInfo[line.sindex]
#                 end = self.cordInfo[line.eindex]
#                 # start-end 직선의 거리
#                 dSquare = (end.x-start.x)**2 + (end.y-start.y)**2
#                 # 중심에서 각 점까지의 거리
#                 d1 = self.cordInfo[line.sindex].distance
#                 d2 = self.cordInfo[line.eindex].distance

#                 angle_radian = acos((d1**2 + d2**2 - dSquare)/(2*d1*d2))
#                 angle = degrees(angle_radian)
                
#                 logList.append(angle)
#             return logList
#         except Exception as e:
#             print(e)

#     # 이상치 제거 후 평균, 표준편차 계산
#     def removeOutlier(self, data:list):
#         mean = np.mean(np.array(data))
#         std = np.std(np.array(data))

#         newData = []
#         for d in data:
#             zScore = (d-mean)/std
#             if np.abs(zScore) <= 1:
#                 newData.append(d)
        
#         newMean = np.mean(np.array(newData))
#         newStd = np.std(np.array(newData))
#         return newMean, newStd

#     # 데이터값의 확률분포 계산
#     def check95(self, mean, std):
#         zScore = 1.96
#         max = mean + zScore * std
#         min = mean - zScore * std

#         return min, max

