# 지도 정보 클래스
from collections import deque
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np
import scandata
import scanLog
from calculate import check95, checkFunc, newCord, removeOutlier
from scandata import Scan
from unit import Cord, Line


# 지도
class Map:
    def __init__(self, scan:Scan):
        self.cordInfo = scan.cordInfo
        self.cordXY = scan.cordXY
        #self.angleInfo = []
        #self.distInfo = []
        self.interInfo = scan.interInfo
        self.funcInfo = scan.funcInfo # 각 line의 기울기 정보
        self.lineInfo = scan.lineInfo # 각 line정보
        self.position = (0,0,0) # 센서의 위치/각도
        self.posLog = [] # 위치 로그
        self.scanLog = deque(maxlen=100)
        self.lineLog = deque(maxlen=100)
    
    # 새로운 스캔 데이터가 입력되었을 때 분석，업데이트
    '''
        scanLog에는 각 스캔 데이터의 원본 좌표값이 삽입됨
        compare로 병합을 비교, 병합된 점 데이터는 cordInfo에 삽입됨
        병합 후 line을 재계산하여 데이터를 lineInfo와 funcInfo에 삽입함
    '''
    def update(self, scan:Scan):
        # Todo: 위치 업데이트
        #self.posLog.append(self.position)
        #self.position = self.findSimiliar(scan)

        # 선 비교, 병합
        # 원본 좌표값, 직선 데이터 로그에 삽입
        self.scanLog.append(self.cordInfo)
        self.lineLog.append(self.lineInfo)

        # 비교, 병합
        self.compare(scan)

    '''
        # icp알고리즘 적용, 지도 데이터 갱신
        # 스캔 데이터 수가 다를 경우 보간
        logScan = self.scanLog[-1]
        newScan = self.matchSize(scan)

        result = best_fit_transform(newScan, logScan)
        T, R, t = result

        # 매핑
        result2 = np.dot(T, np.vstack((newScan.T, np.ones((1, newScan.shape[0])))))

        # 지도 데이터 갱신
        # print(f'addData: {result2.T[:, :2]}')
        self.cordInfo.extend(result2.T[:, :2])

        # 로그에 추가
        self.scanLog.append(result2)
    '''

    # line 데이터를 기반으로 비교, 병합
    def compare(self, scan:Scan):
        mapLine = self.lineInfo # 직선 정보
        mapFunc = self.funcInfo # 기울기 정보
        mapMid =  [((line.startX + line.endX)/2, (line.startY + line.endY)/2) for line in self.lineInfo]
        scanLine = scan.lineInfo
        scanFunc = scan.funcInfo
        scanMid = [((line.startX + line.endX)/2, (line.startY + line.endY)/2) for line in scan.lineInfo]

        # 각 선마다 중간지점의 값과 기울기 데이터로 비교함
        '''
            기울기 비율이 일정 이하인 선 중에서 중간좌표의 거리가 가장 짧은 데이터 검색
            단, 거리 차이가 너무 크다면 적용하지 않음
        '''
        Index = []
        funcDiff = []
        distDiff = []

        # 스캔 데이터의 각 직선마다 최소거리 직선 검색
        for index in range(len(scanLine)):
            start = scanMid[index]
            minIndex = -1
            min = 999999999
            for index2, end in enumerate(mapMid):
                distSquare = (start[0]-end[0])**2 + (start[1]-end[1])**2
                if distSquare < min:
                    minIndex = index2
                    min = sqrt(distSquare)
            Index.append(minIndex)
            funcDiff.append(abs(mapFunc[minIndex]/scanFunc[index]))
            distDiff.append(min)
            
        # 비정상적인 데이터는 그대로, 나머지는 병합
        '''
            기울기 비율, 선의 중점 간 거리를 기준으로 확률분포 계산 후 병합
        '''
        newLineInfo = []
        # fmin, fmax = check95(*removeOutlier(funcDiff))
        # fmin, fmax = checkFunc(funcDiff)
        # dmin, dmax = check95(*removeOutlier(distDiff))
        # for index in range(len(Index)):
        #     if (fmin <= funcDiff[index] <= fmax) & (dmin <= distDiff[index] <= dmax):
        #         # 병합 진행
        #         newLine = self.mergeLine(scanLine[index], mapLine[Index[index]])
        #         newLineInfo.append(newLine)
        #     else:
        #         # 그대로 삽입
        #         newLine = self.mergeLine(scanLine[index])
        #         newLineInfo.append(newLine)

        '''
            기울기 비율이 min과 max사이인 점 중 같은 선에 중복값이 선택 시 거리가 짧은 순으로 배정함
        '''
        
        self.lineInfo = newLineInfo
            

    # 직선 2개를 병합
    '''
        1. 두 직선의 시작점/끝점 간 중점을 통해 기울기 평균을 구한다
        2. 시작점/끝점을 각각 비교하여 직선 밖에 있는 점을 구한다
        3. 해당 점에서 직선에 내린 수선의 발을 새로운 시작점/끝점으로 설정한다
    '''
    def mergeLine(self, newLine:Line, oldLine:Line=None) -> Line:
        # 단순 추가인 경우
        if oldLine == None:
            finalLine = newLine
        # 병합인 경우
        else:
            mid1 = ((oldLine.startX+newLine.startX)/2, (oldLine.startY+newLine.startY)/2)
            mid2 = ((oldLine.endX+newLine.endX)/2, (oldLine.endY+newLine.endY)/2)
            newMid = ((mid1[0]+mid2[0])/2, (mid1[1]+mid2[1])/2)
            newFunc = (mid2[1]-mid1[1])/(mid2[0]-mid1[0]) # 새로운 기울기
            # 새로운 시작 좌표
            newStart = newCord(newFunc, (newLine.startX, newLine.startY), (oldLine.startX, oldLine.startY), mid1, newMid)
            newEnd = newCord(newFunc, (newLine.endX, newLine.endY), (oldLine.endX, oldLine.endY), mid2, newMid)
            
            finalLine = Line(Cord(newStart), Cord(newEnd), 99, 99)

        return finalLine

            

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

if __name__ == '__main__':
    logData = scanLog.loadScanLog()[0]
    instance = Map(scandata.Scan(logData))

    # 정규분포화
    data_array = np.array(instance.interInfo)
    # 최솟값과 최댓값 계산
    min_value = np.min(data_array) - 500
    max_value = np.max(data_array) + 500

    # 최솟값 - 500부터 최댓값 + 500까지의 범위에서 히스토그램 그리기
    plt.hist(data_array, bins=np.arange(min_value, max_value + 50, 50), alpha=0.7, color='blue')

    plt.title('Distribution of Data from {} to {}'.format(min_value, max_value))  # 영문으로 표시
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.show()