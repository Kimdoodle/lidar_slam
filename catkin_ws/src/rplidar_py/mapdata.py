# 지도 정보 클래스
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
    
    # 새로운 스캔 데이터가 들어왔을 때 분석，업데이트
    def update(self, scan:scandata.Scan):
        if self.new:
            self.cordInfo = scan.cordInfo
            self.angleInfo = scan.angleInfo
            self.distInfo = scan.distInfo
            self.interInfo = scan.interInfo
            self.funcInfo = scan.funcInfo
            self.lineInfo = scan.lineInfo


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