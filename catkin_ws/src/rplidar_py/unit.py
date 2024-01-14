# 스캔 데이터의 좌표, 선 클래스
from math import cos, radians, sin

import numpy as np


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

# 좌표의 회전 계산
def rotate_cord(x, y, angle):
    # x' = xcos - ysin, y' = xsin + ycos
    sin0 = sin(radians(angle))
    cos0 = cos(radians(angle))
    x2 = x*cos0 - y*sin0
    y2 = x*sin0 + y*cos0
    
    return (x2, y2)

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
    
    def toString(self):
        return(f'{self.startX}, {self.startY}, {self.endX}, {self.endY}')
