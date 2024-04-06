# gui를 위한 계산 함수
import numpy as np


# 좌표의 비율 조정
def moveCord(cord: tuple, ratio: float) -> tuple:
    x, y = cord
    return x * ratio, y * ratio

# 좌표의 반지름 합차 계산
def ovalCord(cord: tuple, radius: int) -> tuple:
    x, y = cord
    return (x - radius, y - radius), (x + radius, y + radius)

# 좌표 회전
def rotateCord(cord: tuple, rotate: int) -> tuple:
    rad = np.radians(rotate)
    x1, y1 = cord
    x2 = x1 * np.cos(rad) - y1 * np.sin(rad)
    y2 = x1 * np.sin(rad) + y1 * np.cos(rad)

    return x2, y2
