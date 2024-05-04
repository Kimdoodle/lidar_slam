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

    def toLog(self):
        return self.quality, self.angle, self.distance

    def toString(self):
        return f'{self.quality}, {self.angle}, {self.distance}, {self.x}, {self.y}'

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