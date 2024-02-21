import numpy as np
from math import atan,atan2, degrees, sin, cos, radians

# # 좌표를 이동, 회전
# def calculate_move(x, y, movex, movey, rotate) -> tuple:
#     x += movex
#     y += movey
#     angle_rad = np.deg2rad(rotate)
#     # Define rotation matrix
#     rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
#                                 [np.sin(angle_rad), np.cos(angle_rad)]])
#
#     # Define point as a vector
#     point = np.array([x, y])
#
#     # Rotate point
#     rotated_point = np.dot(rotation_matrix, point)
#     rotated_point = np.round(rotated_point, 4)
#
#     return rotated_point


a = (5,0)
print(calculate_move(a[0], a[1], 0, 0, -90))
