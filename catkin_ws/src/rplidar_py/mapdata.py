# 지도 정보 클래스
import bisect
import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import scandata
import scanLog
from icp import best_fit_transform
from scipy.interpolate import interp1d

mapXSize = 5000
mapYSize = 5000
logSize = 100

# 지도
class Map:
    def __init__(self):
        self.new = True
        self.cordInfo = []
        self.cordXY = []
        self.angleInfo = []
        self.distInfo = []
        self.interInfo = []
        self.funcInfo = []
        self.lineInfo = []
        self.position = (0,0,0) # 센서의 위치/각도
        self.posLog = [] # 위치 로그
        self.scanLog = deque(maxlen=100)
    
    # 새로운 스캔 데이터가 입력되었을 때 분석，업데이트
    def update(self, scan:scandata.Scan):
        if self.new:
            self.cordInfo = scan.cordInfo
            self.cordXY = scan.cordXY
            # self.angleInfo = scan.angleInfo
            # self.distInfo = scan.distInfo
            self.interInfo = scan.interInfo
            # self.funcInfo = scan.funcInfo
            # self.lineInfo = scan.lineInfo
            self.scanLog.append(np.array(self.cordXY))
            self.new = False
        else:
            # Todo: 위치 업데이트
            #self.posLog.append(self.position)
            #self.position = self.findSimiliar(scan)
            
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

    # 스캔 데이터를 (x,y)의 리스트로 변경
    def transform(self, data):
        return data.cordXY
    
    # 스캔 데이터 수가 다른 경우 보간
    def matchSize(self, data):
        length = len(self.scanLog[-1])
        x_values, y_values = zip(*self.scanLog[-1])

        int_function = interp1d(x_values, y_values, kind='linear', fill_value='extrapolate')

        int_x = np.linspace(min(x_values), max(x_values), length)
        int_y = int_function(int_x)

        return np.array(list(zip(int_x, int_y)))


    # 새로운 스캔 데이터와 유사한 로그 검색
    '''
        반환값은 추정되는 위치(+ 회전각도)
    '''
    def findSimiliar(self, scan:scandata.Scan):
        # Todo: 현재 디버깅을 위해 위치는 현재 position을 반환
        estimatePos = self.position
        # 중심점을 정한 후 좌표 갱신
        for cord in scan.cordInfo:
            cord.move_xy(estimatePos[0], estimatePos[1])

        return estimatePos

    # 로그 갱신 - 최대 크기 유지
    def addLog(self, data):
        self.scanLog.append(data)

    # 초기화함수
    def reset(self):
        self.__init__()

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