# 스캔 데이터 클래스
import numpy as np

class Scan:
    def __init__(self, data):
        self.scanInfo = [(ele[0], ele[1], ele[2]) for ele in data]
        self.cordInfo = [] # 각 점의 x, y좌표(중심 기준)
        self.interInfo = []  # 각 점 사이의 거리차이
        self.angleInfo = []  # 각 점 사이의 각도차이
        self.distInfo = []  # 중심에서 각 점까지의 거리 차이
        # self.funcInfo = []  # 두 점을 연결한 직선의 기울기행

    # 후처리
    def postProcess(self):
        self.calXY()
        self.calInfo()

    # 후처리 - 좌표 계산
    def calXY(self):
        for data in self.scanInfo:
            angle = np.radians(data[1])
            dist = data[2]
            self.scanInfo.append(
                [dist * np.cos(angle), dist * np.sin(angle)]
            )
            
    # 후처리 - 학습 데이터 계산
    def calInfo(self):
        length = len(self.scanInfo)
        for i, info1, cord1 in \
            enumerate(zip(self.scanInfo, self.cordInfo)):
            info2 = self.scanInfo[(i+1)/length]
            cord2 = self.cordInfo[(i+1)/length]
            
            # interInfo
            deltaX = cord2[0] - cord1[0]
            deltaY = cord2[1] - cord1[1]
            self.interInfo.append(np.sqrt(deltaX**2 + deltaY**2))

            # angleInfo
            self.angleInfo.append(np.abs(info2[1] - info1[1]))

            # distInfo
            self.distInfo.append(np.abs(info2[2] - info1[2]))

            # # funcInfo
            # self.funcInfo.append(deltaY/deltaX)
    
