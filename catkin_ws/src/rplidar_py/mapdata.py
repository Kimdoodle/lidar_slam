# 지도 정보 클래스
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
        self.position = (0,0) # 센서의 위치
        self.scanLog = [] # 로그파일
    
    # 새로운 스캔 데이터가 들어왔을 때 분석，업데이트
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
            pass
        
        # 스캔 데이터 단순화
        self.simplify(scan)
        # 스캔 데이터를 로그에 저장
        self.scanLog.append((scan, self.makeLog(scan)))

    # 스캔 데이터 단순화 - 직선 수를 줄임
    '''
        
    '''
    def simplify(self, scan:scandata.Scan):
        pass

    # 스캔 파일 로그화 - 각 직선과 직선중점 - 중심의 각도를 중심으로.
    def makeLog(self, scan:scandata.Scan):
        logList = []
        try:
            for line in scan.lineInfo:
                start = scan.cordInfo[line.sindex]
                end = scan.cordInfo[line.eindex]
                # start-end 직선의 거리
                d = math.sqrt((end.x-start.x)**2 + (end.y-start.y)**2)
                # 중심에서 각 점까지의 거리
                d1 = scan.cordInfo[line.sindex].distance
                d2 = scan.cordInfo[line.eindex].distance

                angle_radian = math.acos((d1**2 + d2**2 - d**2)/(2*d1*d2))
                angle = math.degrees(angle_radian)
                
                logList.append(angle)
            print(logList)
            #debug
            mean = np.mean(np.array(logList))
            std = np.std(np.array(logList))
            max = mean + 1.96 * std
            min = mean - 1.96 * std
            print(f'직선 정보의 평균: {mean}, 표준편차: {std}')

            temp = []
            i = 0
            angle = logList[i]
            while True:
                i = (i+1)%len(logList)
                angle2 = logList[i]
                temp.append(angle2/angle)
                angle = angle2
                if i == 0:
                    mean, std = scan.removeOutlier(temp)
                    max = mean + 0.5 * std
                    min = mean - 0.5 * std
                    print(f'나눈 정보의 평균: {mean}, 표준편차: {std}')
                    print(f'범위: {min} to {max}')
                    print(temp)
                    break
            
            return logList
        except Exception as e:
            print(e)


    # 새로운 스캔 데이터와 유사한 로그 검색
    '''
        
    '''
    def findSimiliar(self, scan:scandata.Scan):
        pass



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