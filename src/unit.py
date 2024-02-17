# 스캔 데이터의 좌표, 선 클래스
from math import cos, radians, sin
import warnings
import numpy as np
from calculate import calculate_move

warnings.filterwarnings("error", category=RuntimeWarning)

# 좌표
class Cord:
    def __init__(self, data):
        if len(data) == 3:
            self.quality = data[0]
            self.angle = data[1]
            self.distance = data[2]
            self.x = self.distance * np.cos(np.radians(self.angle))
            self.y = self.distance * np.sin(np.radians(self.angle))
        elif len(data) == 2:
            self.x = data[0]
            self.y = data[1]

    def move_angle(self, angle):
        sin0 = sin(radians(angle))
        cos0 = cos(radians(angle))
        self.x = self.x*cos0 - self.y*sin0
        self.y = self.x*sin0 + self.y*cos0

    def toLog(self):
        return (self.quality, self.angle, self.distance)
    def toString(self):
        return(f'{self.quality}, {self.angle}, {self.distance}, {self.x}, {self.y}')
    
    # 대소비교
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)
    def __le__(self, other):
        return (self.x, self.y) <= (other.x, other.y)
    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)
    def __ne__(self, other):
        return (self.x, self.y) != (other.x, other.y)
    def __gt__(self, other):
        return (self.x, self.y) > (other.x, other.y)
    def __ge__(self, other):
        return (self.x, self.y) >= (other.x, other.y)

# 선
class Line:
    def __init__(self, cordI:Cord, cordII:Cord, indexI:int, indexII:int):
        self.cordI = cordI
        self.cordII = cordII
        self.sindex = indexI
        self.eindex = indexII
        self.startX = cordI.x
        self.startY = cordI.y
        self.endX = cordII.x
        self.endY = cordII.y
        self.func = self.calFunc()
        self.mid: tuple = self.calMid()
    
    def calFunc(self):
        try:
            if self.endX - self.startX  == 0:
                return 0
            # print(self.startX, self.startY, self.endX, self.endY)
            a = (self.endY - self.startY)/(self.endX - self.startX)
            return a
        except RuntimeWarning as rw:
            print("runtimewarning")
    
    def calMid(self):
        return (self.endX - self.startX) / 2, (self.endY - self.startY) / 2
    
    def toString(self):
        return f'{self.startX}, {self.startY}, {self.endX}, {self.endY}'

    # 선을 각도/이동벡터 만큼 움직임
    def move(self, angle, t):
        self.startX, self.startY = calculate_move(self.startX, self.startY, t[0], t[1], angle)
        self.endX, self.endY = calculate_move(self.endX, self.endY, t[0], t[1], angle)
        self.func = self.calFunc()
        self.mid = self.calMid()