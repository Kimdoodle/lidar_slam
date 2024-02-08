# 지도 정보 클래스
from collections import deque
from math import atan, sqrt

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

import scandata
import scanLog
from calculate import (calculate_angle, calculate_dist, check95, checkFunc,
                       newCord, removeOutlier, calculateVariance, calculate_move)
from scanCheck import MoveLine, DataCheck
from scandata import Scan
from unit import Cord, Line
from icp import icp, decompose_ICP


# 지도
class Map:
    def __init__(self, scan: Scan):
        # 초기 스캔 데이터
        self.scanDataLog = [scan]

        # 커서 위치 로그
        self.moveLog = [(0, 0)]

        # My
        self.myStep0_cluster = [scan.clusters]  # 클러스터링 데이터
        self.myStep1_makeLine = [scan.lineInfo]  # 선 생성 후 각각의 선 데이터
        self.myStep2_ICPResult = [scan.lineInfo]  # 선 조합 생성 후 데이터, 2개의 lineInfo List
        self.myStep3_MergedData = []  # 선 조합 결합 후 데이터, 1개의 lineInfo List

        self.myStep99_temp = []  # 더미데이터


    # 새로운 데이터 업데이트
    def update(self, data):
        scanData = Scan(data)
        self.scanDataLog.append(scanData)
        self.myStep0_cluster.append(scanData.clusters)
        self.myStep1_makeLine.append(scanData.lineInfo)
        # self.my1_lineICP(self.myStep2_ICPResult[0], scanData.lineInfo)

    # ICP알고리즘을 이용해 orig와 new간 선 조합의 이동
    def my1_lineICP(self, orig, new):
        origMid = [line.mid for line in orig]
        newMid = [line.mid for line in new]

        tree = cKDTree(origMid)
        comps = [tree.query(mid, k=1)[1] for mid in newMid]
        # 언더샘플링
        comps = self.underSample(comps, len(origMid))
        newMid2 = [newMid[ele] for ele in comps]
        newMid2 = np.array(newMid2)

        # comps와 newMid에 ICP적용하여 회전, 이동값 도출
        origMid2 = np.array(origMid)
        T, distances, i = icp(np.array(origMid2), newMid2)
        angle, t = decompose_ICP(T)

        # new의 모든 선에 대하여 회전, 이동 시행
        for line in new:
            line.move(angle, t)
        curPos = self.moveLog[-1]
        newPos = calculate_move(curPos[0], curPos[1], -t[0], -t[1], -angle)
        self.moveLog.append(newPos)
        self.myStep2_ICPResult = [orig, new]

    def underSample(self, comp: list, target: int) -> list:
        reduce = abs(len(comp) - target)
        cur = 0
        flag = False
        temp = []
        log = []
        # 1. 연속되지 않은 값 우선적으로 삭제
        for index, ele in enumerate(comp):
            if ele not in log:
                temp.append(ele)
            elif comp[index-1] == comp[index+1] and comp[index-1] != comp[index] and flag == False:
                temp.append(comp[index-1])
                cur += 1
                if cur == reduce: flag = True
            else:
                temp.append(ele)

        # 2.

        return temp

    '''
        선 조합 생성 방법
        1. 회전값 구하기
        --> 가장 가까운 중점을 가진 선과의 기울기 차이 확률분포 계산
        --> 이상치 제거 후 평균을 내어 회전값 도출
        
        2. 이동값 구하기
        --> 가장 가까운 중점과의 거리차이 계산
        --> 이상치 제거 후 평균을 내어 이동값 도출
    '''
    def calculate_Rotate(self, orig, new):
        origMid = [line.mid for line in orig]
        origFunc = [line.func for line in orig]
        newMid = [line.mid for line in new]
        newFunc = [line.func for line in new]

        tree = cKDTree(origMid)
        dist = [tree.query(mid, k=1)[1] for mid in newMid]
        compFunc = []
        for i, index in enumerate(dist):
            f1 = newFunc[i]
            f2 = origFunc[index]
            compFunc.append(calculate_angle(f1, f2))
        
        funcOutlier = removeOutlier(compFunc)
        funcRes = np.mean(np.array(funcOutlier))
        
        compDist = []
        for i, index in enumerate(dist):
            d1 = newMid[i]
            d2 = origMid[index]
            compDist.append(calculate_dist(d1, d2))

        compFunc = [origFunc[index] for index in dist]  # 기울기

    # 이상치 제거 후 평균 반환
    def removeOutlier(data: list):
        mean = np.mean(np.array(data))
        std = np.std(np.array(data))

        newData = []
        for d in data:
            zScore = (d - mean) / std
            if abs(zScore) <= 1:
                newData.append(d)

        newMean = np.mean(np.array(newData))
        newStd = np.std(np.array(newData))
        return newMean, newStd

