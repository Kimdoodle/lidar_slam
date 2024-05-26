import numpy as np

# 좌표
class Cord:
    def __init__(self, data):
        self.quality = data[0]
        self.angle = data[1]
        self.distance = data[2]
        self.x = self.distance * np.cos(np.radians(self.angle))
        self.y = self.distance * np.sin(np.radians(self.angle))