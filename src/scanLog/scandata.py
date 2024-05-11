import numpy as np

class Scan:
    def __init__(self, data):
        self.scanInfo = np.array([(ele[0], np.radians(ele[1]), ele[2]) for ele in data])
        self.length = len(self.scanInfo)
        self.cordInfo = None
        self.x = np.zeros(self.length)  # X 좌표
        self.y = np.zeros(self.length)  # Y 좌표

        self.interInfoLeft = np.zeros(self.length)
        self.interInfoRight = np.zeros(self.length)
        self.angleInfoLeft = np.zeros(self.length)
        self.angleInfoRight = np.zeros(self.length)
        self.distInfoLeft = np.zeros(self.length)
        self.distInfoRight = np.zeros(self.length)

    def polar_to_cartesian(self):
        # 극좌표를 카테시안 좌표로 변환
        self.x, self.y = self.scanInfo[:, 2] * np.cos(self.scanInfo[:, 1]), self.scanInfo[:, 2] * np.sin(self.scanInfo[:, 1])

    def postProcess(self):
        # 좌표 정보 계산
        self.polar_to_cartesian()
        self.calInfo()

    def calInfo(self):
        for i in range(self.length):
            x1, y1 = self.x[i], self.y[i]

            if i == 0:
                x0, y0 = self.x[-1], self.y[-1]
            else:
                x0, y0 = self.x[i-1], self.y[i-1]

            if i == self.length - 1:
                x2, y2 = self.x[0], self.y[0]
            else:
                x2, y2 = self.x[i+1], self.y[i+1]

            # 거리 정보 계산
            self.interInfoLeft[i] = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            self.interInfoRight[i] = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            # 각도 정보 계산 (간단화를 위해 각도 데이터는 scanInfo에서 직접 접근)
            self.angleInfoLeft[i] = np.abs(self.scanInfo[i][1] - self.scanInfo[i-1][1])
            self.angleInfoRight[i] = np.abs(self.scanInfo[(i+1) % self.length][1] - self.scanInfo[i][1])

            # 중심 거리 정보 계산
            self.distInfoLeft[i] = np.abs(self.scanInfo[i][2] - self.scanInfo[i-1][2])
            self.distInfoRight[i] = np.abs(self.scanInfo[(i+1) % self.length][2] - self.scanInfo[i][2])
