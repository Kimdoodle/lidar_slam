# 지도 정보 클래스
from collections import deque
from math import atan, sqrt

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

import scandata
import scanLog
from calculate import (calculate_angle, calculate_dist, check95, checkFunc,
                       newCord, removeOutlier_integer, removeOutlier_tuple, calculateVariance, calculate_move,
                       sort_cords)
from scanCheck import MoveLine, DataCheck
from scandata import Scan
from unit import Cord, Line
from icp import icp, decompose_ICP
from copy import deepcopy

# 지도
class Map:
    def __init__(self, scan: Scan):
        # 현재 지도 데이터
        self.cordInfo = scan.cordInfo

        # 로그
        self.scanLog = [scan]  # 지도 데이터에 삽입된 스캔 데이터들을 저장
        self.clusterLog = [scan.clusters]   # 클러스터링 로그
        self.lineLog = [scan.lineInfo]   # 선 로그
        self.moveLog = [(0, 0, 0)]  # 중심 이동 로그

        # My Algorithm
        self.myStep0_cluster = [scan.clusters]  # 클러스터링 데이터
        self.myStep1_makeLine = [scan.lineInfo]  # 선 생성 후 각각의 선 데이터
        self.myStep2_ICPResult = [scan.lineInfo]  # 선 조합 생성 후 데이터, 2개의 lineInfo List
        self.myStep3_MergedData = None  # 선 조합 결합 후 데이터, 1개의 lineInfo List

        self.myStep99_temp = []  # 더미데이터


    # 새로운 데이터 업데이트
    def update(self, data):
        # 새로운 데이터를 Scan클래스 형태로 생성
        scanData = Scan(data)

        # 로그에 신규 데이터 추가
        self.scanLog.append(scanData)
        self.clusterLog.append(scanData.clusters)
        self.lineLog.append(scanData.lineInfo)

        # odometry 계산
        self.calculate_odometry(self.lineLog[-2], self.lineLog[-1])

        # 계산한 odometry적용, 병합 진행
        self.merge()


    # 선 조합 생성
    def calculate_odometry(self, orig, new):
        """
            1. 회전값 구하기
            --> 가장 가까운 중점을 가진 선과의 기울기 차이 확률분포 계산
            --> 이상치 제거 후 평균을 내어 회전값 도출

            2. 이동값 구하기
            --> 가장 가까운 중점과의 거리차이 계산
            --> 이상치 제거 후 평균을 내어 이동값 도출
        """
        origMid = [line.mid for line in orig]
        origFunc = [line.func for line in orig]
        newMid = [line.mid for line in new]
        newFunc = [line.func for line in new]

        tree = cKDTree(origMid)
        dist = [tree.query(mid, k=1)[1] for mid in newMid]
        compFunc = []  # 기울기
        for i, index in enumerate(dist):
            f1 = newFunc[i]
            f2 = origFunc[index]
            compFunc.append(f2-f1)

        funcRes = removeOutlier_integer(compFunc)[0]

        compDist = []  # 거리
        for i, index in enumerate(dist):
            d1 = newMid[i]
            d2 = origMid[index]
            compDist.append((abs(d1[0]-d2[0]), abs(d1[1]-d2[1])))

        distRes = removeOutlier_tuple(compDist)[0]

        self.moveLog.append((distRes[0], distRes[1], -funcRes))

    # 추정한 odometry를 이용해 최종 병합
    def merge(self):
        """
            1. 신규 데이터의 모든 점에 회전/이동 적용
            2. 기존 점 데이터와 합친 후 클러스터링/선 생성
        """

        # 1. 신규 데이터의 모든 점에 회전/이동 적용
        scan = deepcopy(self.scanLog[-1])  # 스캔된 데이터
        move = (self.moveLog[-1][0], self.moveLog[-1][1])  # 이동
        angle = self.moveLog[-1][2]    # 회전
        origLog = scan.cordXY
        newLog = [calculate_move(l[0], l[1], move[0], move[1], angle) for l in origLog]

        # 2. 기존 점 데이터와 합친 후 축소, 클러스터링/선 생성
        scan.cordInfo = self.cordInfo + [Cord[l] for l in newLog]
        scan.cordInfo = sort_cords(scan.cordInfo)
        scan.calculate()
        scan.makeLine()
        self.cordInfo = scan.cordInfo
        self.myStep3_MergedData = scan.lineInfo


        # cordInfo = self.scanDataLog[-1].cordInfo
        # cordInfo2 = []
        # for cord in cordInfo:
        #     cord2 = calculate_move(cord.x, cord.y, -move[0], -move[1], -move[2])
        #     cordInfo2.append(cord2)


    # # ICP알고리즘을 이용해 orig와 new간 선 조합의 이동
    # def my1_lineICP(self, orig, new):
    #     origMid = [line.mid for line in orig]
    #     newMid = [line.mid for line in new]
    #
    #     tree = cKDTree(origMid)
    #     comps = [tree.query(mid, k=1)[1] for mid in newMid]
    #     # 언더샘플링
    #     comps = self.underSample(comps, len(origMid))
    #     newMid2 = [newMid[ele] for ele in comps]
    #     newMid2 = np.array(newMid2)
    #
    #     # comps와 newMid에 ICP적용하여 회전, 이동값 도출
    #     origMid2 = np.array(origMid)
    #     T, distances, i = icp(np.array(origMid2), newMid2)
    #     angle, t = decompose_ICP(T)
    #
    #     # new의 모든 선에 대하여 회전, 이동 시행
    #     for line in new:
    #         line.move(angle, t)
    #     curPos = self.moveLog[-1]
    #     newPos = calculate_move(curPos[0], curPos[1], -t[0], -t[1], -angle)
    #     self.moveLog.append(newPos)
    #     self.myStep2_ICPResult = [orig, new]
    #
    # def underSample(self, comp: list, target: int) -> list:
    #     reduce = abs(len(comp) - target)
    #     cur = 0
    #     flag = False
    #     temp = []
    #     log = []
    #     # 1. 연속되지 않은 값 우선적으로 삭제
    #     for index, ele in enumerate(comp):
    #         if ele not in log:
    #             temp.append(ele)
    #         elif comp[index-1] == comp[index+1] and comp[index-1] != comp[index] and flag == False:
    #             temp.append(comp[index-1])
    #             cur += 1
    #             if cur == reduce: flag = True
    #         else:
    #             temp.append(ele)
    #
    #     # 2.
    #
    #     return temp
    #
    #

