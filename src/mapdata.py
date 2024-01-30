# 지도 정보 클래스
from collections import deque
from math import atan, sqrt

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

import scandata
import scanLog
from calculate import (calculate_angle, calculate_dist, check95, checkFunc,
                       newCord, removeOutlier)
from scanCheck import MoveLine, DataCheck
from scandata import Scan
from unit import Cord, Line
from icp import best_fit_transform


# 지도
class Map:
    def __init__(self, scan: Scan):
        self.initScan = scan
        # self.cordInfo = scan.cordInfo
        # self.cordXY = scan.cordXY
        # self.angleInfo = scan.angleInfo
        # self.distInfo = scan.distInfo
        # self.interInfo = scan.interInfo
        # self.funcInfo = scan.funcInfo  # 각 line의 기울기 정보

        self.lineInfo = scan.lineInfo  # 각 line정보
        self.position = (0, 0, 0)  # odometry

        # 초기 스캔 데이터
        self.scanDataLog = [self.initScan]

        # 과정 데이터 로그
        # ICP
        self.icpStep1_DownSample = []  # 다운샘플링 데이터, 2개의 (x,y) list
        self.icpStep2_ICPResult = ()  # ICP알고리즘 결과 구해진 T,R,t
        self.icpStep3_MergedData = []  # ICP알고리즘 결과 합쳐진 1개의 (x,y) list

        # My Algorithm
        self.myStep0_cluster = [self.initScan.clusters] # 클러스터링 데이터
        self.myStep1_makeLine = [self.lineInfo]  # 선 생성 후 각각의 선 데이터
        self.myStep2_lineComp = []  # 선 조합 생성 후 데이터, 2개의 lineInfo List
        self.myStep3_MergedData = []  # 선 조합 결합 후 데이터, 1개의 lineInfo List


    # 새로운 데이터를 이용하여 업데이트
    def update(self, data):
        scanData = Scan(data)
        self.scanDataLog.append(scanData)
        self.myStep0_cluster.append(scanData.clusters)
        self.myStep1_makeLine.append(scanData.lineInfo)

        # ICP알고리즘 적용
        # self.calculate_ICP(data)

#     # ICP알고리즘
#     def calculate_ICP(self, data: Scan):
#         # Step1 - 데이터 다운샘플링
#         # 2개의 cordInfo를 비교하여 큰 쪽을 다운샘플링
#         flag = 'self'  # flag - 어느쪽을 다운샘플링했는가를 구별
#         if len(self.cordInfo) > len(data.cordInfo):
#             new = downSample(self.cordInfo, self.angleInfo, len(self.cordInfo) - len(data.cordInfo))
#             new = [(cord.x, cord.y) for cord in new]
#             self.icpStep1_DownSample = [new, [(cord.x, cord.y) for cord in data.cordInfo]]
#         else:
#             flag = 'new'
#             new = downSample(data.cordInfo, data.angleInfo, len(data.cordInfo) - len(self.cordInfo))
#             new = [(cord.x, cord.y) for cord in new]
#             self.icpStep1_DownSample = [new, [(cord.x, cord.y) for cord in self.cordInfo]]
#
#         # Step2 - ICP알고리즘 계산
#         array1 = np.array(self.icpStep1_DownSample[0])
#         array2 = np.array(self.icpStep1_DownSample[1])
#         if flag != 'self':
#             temp = array1
#             array1 = array2
#             array2 = temp
#             # array1에 array2를 매칭
#         T, R, t = best_fit_transform(array2, array1)
#         self.icpStep2_ICPResult = (T, R, t)
#
#         # Step3 - ICP알고리즘 적용
#         newArray2 = np.dot(T, np.vstack((array2.T, np.ones(array2.shape[0])))).T[:, :2]
#         self.icpStep3_MergedData = array1.tolist() + newArray2.tolist()
#
    # My Algorithm
    # Step2 - 선 조합 생성
    def my1_lineComp(self, orig, new):
        # 중심점을 이용해 최단거리 점 2개를 검색
        origMid = [line.mid for line in orig]
        newMid = [line.mid for line in new]
        comps = []
        tree = cKDTree(origMid)
        for mid in newMid:
            indexes = tree.query(mid, k=2)[1]
            # 기울기 차가 적은 데이터를 선택함
            a = calculate_angle(mid.func, orig[indexes[0]].func)
            b = calculate_angle(mid.func, orig[indexes[1]].func)
            comps.append(indexes[0] if a<b else indexes[1])

        print(comps)





#
#
# # ICP알고리즘 - 데이터 다운샘플링
# def downSample(cordInfo: list, angleInfo: list, targetCount: int):
#     # 좌우 각도차이를 비교하여 양쪽 값이 가장 작은 데이터부터 제거
#     angleMean = []
#     for index, angle in enumerate(angleInfo):
#         prev = angleInfo[index - 1]
#         curr = angle
#         angleMean.append((prev + curr) / 2)
#
#     deletes = sorted(range(len(angleMean)), key=lambda k: angleMean[k])[:targetCount]
#     return [cordInfo[i] for i in range(len(angleMean)) if i not in deletes]

    # # 새로운 스캔 데이터가 입력되었을 때 분석，업데이트
    # '''
    #     scanLog에는 각 스캔 데이터의 원본 좌표값이 삽입됨
    #     compare로 병합을 비교, 병합된 점 데이터는 cordInfo에 삽입됨
    #     병합 후 line을 재계산하여 데이터를 lineInfo와 funcInfo에 삽입함
    # '''
    # # 데이터 추가
    # def update(self, scan: Scan):
    #     # 원본 좌표값, 직선 데이터 로그에 삽입
    #     self.scanLog.append(scan)
    #     self.lineLog.append(scan.lineInfo)
    #
    # # 선택한 데이터까지 병합
    # def merge(self, index):
    #     # Todo: 위치 업데이트
    #     # self.posLog.append(self.position)
    #     # self.position = self.findSimiliar(scan)
    #
    #     # 병합
    #     for i in range(len(index)):
    #         if i == 0:
    #             self.cordInfo = self.scanLog[i].cordInfo
    #         else:
    #             self.updateICP(self, self.scanLog[i])
    #
    # # line 데이터를 기반으로 비교, 병합
    # def compare(self, scan: Scan):
    #     '''기존(병합될)데이터'''
    #     mapLine = self.lineInfo  # 직선 정보
    #     mapFunc = [line.func for line in mapLine]  # 각 직선의 기울기 정보
    #     mapMid = [((line.startX + line.endX) / 2, (line.startY + line.endY) / 2) for line in
    #               self.lineInfo]  # 직선의 중점 리스트
    #     '''신규(병합할)데이터'''
    #     scanLine = scan.lineInfo
    #     scanFunc = [line.func for line in scanLine]
    #     scanMid = [((line.startX + line.endX) / 2, (line.startY + line.endY) / 2) for line in scan.lineInfo]
    #
    #     # 신규 중점과 기존 데이터의 중점 조합
    #     # 거리와 기울기를 비교함
    #     # 근접한 좌표 2개를 검색
    #     def find(tuple, kdTree):
    #         _, indexes = kdTree.query(tuple, k=2)
    #         return indexes
    #
    #     comp = []
    #     kdTree = cKDTree(mapMid)
    #     for index, mid in enumerate(scanMid):
    #         partners = find(mid, kdTree)
    #         # 2개의 직선 중 기울기가 비슷한 쪽으로 결정
    #         sFunc = scanFunc[index]
    #         mFunc1 = calculate_angle(sFunc, mapFunc[partners[0]])
    #         mFunc2 = calculate_angle(sFunc, mapFunc[partners[1]])
    #         # 두 직선 모두 적합하지 않은 경우도 고려
    #         if (mFunc1 > 15) & (mFunc2 > 15):
    #             comp.append(-99)
    #         else:
    #             result = 1 if mFunc1 < mFunc2 else 2
    #             comp.append(partners[0] if result == 1 else partners[1])
    #
    #     # 같은 인덱스를 가리키는 선들은 병합
    #     mergedLine = []
    #     compareIndex = 0
    #     for i in comp:
    #         if i != -99:
    #             compareIndex = i
    #             break
    #     temp = []
    #     for i, index in enumerate(comp):
    #         if (len(mergedLine) == 0) & (index == -99):
    #             continue
    #
    #         if (index != compareIndex) & (len(temp) != 0):
    #             # 병합
    #             start = temp[0]
    #             end = temp[-1]
    #             mergedLine.append((index, Line(start.cordI, end.cordII, start.sindex, end.eindex)))
    #             temp = []
    #             compareIndex = index
    #
    #         if index != -99:
    #             temp.append(scanLine[i])
    #     start = temp[0]
    #     end = temp[-1]
    #     mergedLine.append((index, Line(start.cordI, end.cordII, start.sindex, end.eindex)))
    #
    #     # 선 데이터 이동 및 추가
    #     lines = []
    #     moveInfo = MoveLine()
    #     for index, newLine in mergedLine:
    #         if index == -99: continue
    #         self.lineInfo[index] = moveInfo.update(self.lineInfo[index], newLine)
    #     for index, newLine in mergedLine:
    #         if index == -99:
    #             self.lineInfo.append(moveInfo.justMove(newLine))

###########################################################################
# # 각 선마다 중간지점의 값과 기울기  데이터로 비교함
# '''
#     기울기 비율이 일정 이하인 선 중에서 중간좌표의 거리가 가장 짧은 데이터 검색
#     단, 거리 차이가 너무 크다면 적용하지 않음
# '''
# Index = []
# funcDiff = []
# distDiff = []

# # 스캔 데이터의 각 직선마다 최소거리 직선 검색
# for index in range(len(scanLine)):
#     start = scanMid[index]
#     minIndex = -1
#     min = 999999999
#     for index2, end in enumerate(mapMid):
#         distSquare = (start[0]-end[0])**2 + (start[1]-end[1])**2
#         if distSquare < min:
#             minIndex = index2
#             min = sqrt(distSquare)
#     Index.append(minIndex)
#     funcDiff.append(abs(mapFunc[minIndex]/scanFunc[index]))
#     distDiff.append(min)

# # 비정상적인 데이터는 그대로, 나머지는 병합
# '''
#     기울기 비율, 선의 중점 간 거리를 기준으로 확률분포 계산 후 병합
# '''
# newLineInfo = []
# # fmin, fmax = check95(*removeOutlier(funcDiff))
# # fmin, fmax = checkFunc(funcDiff)
# # dmin, dmax = check95(*removeOutlier(distDiff))
# # for index in range(len(Index)):
# #     if (fmin <= funcDiff[index] <= fmax) & (dmin <= distDiff[index] <= dmax):
# #         # 병합 진행
# #         newLine = self.mergeLine(scanLine[index], mapLine[Index[index]])
# #         newLineInfo.append(newLine)
# #     else:
# #         # 그대로 삽입
# #         newLine = self.mergeLine(scanLine[index])
# #         newLineInfo.append(newLine)

# '''
#     기울기 비율이 min과 max사이인 점 중 같은 선에 중복값이 선택 시 거리가 짧은 순으로 배정함
# '''

# self.lineInfo = newLineInfo

# # 직선 2개를 병합
# '''
#     1. 두 직선의 시작점/끝점 간 중점을 통해 기울기 평균을 구한다
#     2. 시작점/끝점을 각각 비교하여 직선 밖에 있는 점을 구한다
#     3. 해당 점에서 직선에 내린 수선의 발을 새로운 시작점/끝점으로 설정한다
# '''
# def mergeLine(self, newLine:Line, oldLine:Line=None) -> Line:
#     # 단순 추가인 경우
#     if oldLine == None:
#         finalLine = newLine
#     # 병합인 경우
#     else:
#         mid1 = ((oldLine.startX+newLine.startX)/2, (oldLine.startY+newLine.startY)/2)
#         mid2 = ((oldLine.endX+newLine.endX)/2, (oldLine.endY+newLine.endY)/2)
#         newMid = ((mid1[0]+mid2[0])/2, (mid1[1]+mid2[1])/2)
#         newFunc = (mid2[1]-mid1[1])/(mid2[0]-mid1[0]) # 새로운 기울기
#         # 새로운 시작 좌표
#         newStart = newCord(newFunc, (newLine.startX, newLine.startY), (oldLine.startX, oldLine.startY), mid1, newMid)
#         newEnd = newCord(newFunc, (newLine.endX, newLine.endY), (oldLine.endX, oldLine.endY), mid2, newMid)

#         finalLine = Line(Cord(newStart), Cord(newEnd), 99, 99)

#     return finalLine

# # 스캔 데이터를 (x,y)의 리스트로 변경
# def transform(self, data):
#     return data.cordXY

# # 스캔 데이터 수가 다른 경우 보간
# def matchSize(self, data):
#     length = len(self.scanLog[-1])
#     x_values, y_values = zip(*self.scanLog[-1])

#     int_function = interp1d(x_values, y_values, kind='linear', fill_value='extrapolate')

#     int_x = np.linspace(min(x_values), max(x_values), length)
#     int_y = int_function(int_x)

#     return np.array(list(zip(int_x, int_y)))

# # 새로운 스캔 데이터와 유사한 로그 검색
# '''
#     반환값은 추정되는 위치(+ 회전각도)
# '''
# def findSimiliar(self, scan:scandata.Scan):
#     # Todo: 현재 디버깅을 위해 위치는 현재 position을 반환
#     estimatePos = self.position
#     # 중심점을 정한 후 좌표 갱신
#     for cord in scan.cordInfo:
#         cord.move_xy(estimatePos[0], estimatePos[1])

#     return estimatePos

# # 로그 갱신 - 최대 크기 유지
# def addLog(self, data):
#     self.scanLog.append(data)

# # 초기화함수
# def reset(self):
#     self.__init__()
