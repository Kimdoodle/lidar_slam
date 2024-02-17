from math import atan,atan2, degrees, sin, cos, radians

# 좌표를 이동, 회전
def calculate_move(x, y, movex, movey, rotate) -> tuple:
    x += movex
    y += movey
    theta = radians(rotate)
    x = cos(theta) * x - sin(theta) * y
    y = sin(theta) * x + cos(theta) * y
    return (x, y)


a = (4,3)
b = (5,0)
funcA = atan2(a[1], a[0])
funcB = atan2(b[1], b[0])
print(funcA)
print(funcB)
print(calculate_move(a[0], a[1], 0, 0, funcB-funcA))
