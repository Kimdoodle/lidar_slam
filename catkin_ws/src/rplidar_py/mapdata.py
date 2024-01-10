# 지도 정보 클래스
import bisect
import math

import matplotlib.pyplot as plt
import numpy as np
import scandata
import scanLog

mapXSize = 5000
mapYSize = 5000

# 지도
class Map:
    def __init__(self):
        self.new = True
        self.cordInfo = []
        self.angleInfo = []
        self.distInfo = []
        self.interInfo = []
        self.funcInfo = []
        self.lineInfo = []
        self.position = (0,0,0) # 센서의 위치/각도
        self.posLog = [] # 위치 로그
        self.scanLog = [] # 로그파일
    
    # 새로운 스캔 데이터가 입력되었을 때 분석，업데이트
    def update(self, scan:scandata.Scan):
        if self.new:
            self.cordInfo = scan.cordInfo
            self.angleInfo = scan.angleInfo
            self.distInfo = scan.distInfo
            self.interInfo = scan.interInfo
            self.funcInfo = scan.funcInfo
            self.lineInfo = scan.lineInfo
            self.new = False
        else:
            # 위치 업데이트
            self.posLog.append(self.position)
            self.position = self.findSimiliar(scan)
            
            # 좌표 정보를 적합한 위치에 삽입
            for cord in scan.cordInfo:
                index = bisect.bisect_left(self.cordInfo, cord)
                self.cordInfo.insert(index, cord)



    # 새로운 스캔 데이터와 유사한 로그 검색
    '''
        반환값은 추정되는 위치(+ 회전각도)
    '''
    def findSimiliar(self, scan:scandata.Scan):
        # Todo: 현재 디버깅을 위해 위치는 현재 position을 반환
        estimatePos = self.position
        # 중심점을 정한 후 좌표 갱신
        for cord in scan.cordInfo:
            cord.updateCord(estimatePos[0], estimatePos[1])

        return estimatePos

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