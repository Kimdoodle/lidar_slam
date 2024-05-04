# 로그 데이터 스캔
import os
import re

import numpy as np


# 로그 파일을 스캔하여 좌표 데이터 반환
def load(path: str) -> list:
    try:
        # 파일 목록을 구성
        fileList = os.listdir(path)

        # 파일 데이터 반환
        fileData = []
        for file in fileList:
            text = open(path + '/' + file, 'r').read()
            data = re.findall(r'\((.*?)\)', text)

            # tuple 형태로 저장
            data_list = []
            for item in data:
                ele = tuple(map(float, item.split(',')))
                data_list.append(ele[1:])
            fileData.append(data_list)

        return fileData
    except Exception as e:
        print(e)


'''
csv파일 column
angleDiff / distanceDiff / distanceFromCenterDiff
'''


# 로그데이터를 계산하여 csv파일로 저장
def text2Csv(dataList: list):
    # data는 각 파일별 데이터를 삽입한 list의 형태
    for data in dataList:
        length = len(data)
        info = []
        for i in range(length):
            dot1 = data[i]
            dot2 = data[(i + 1) % length]
            angleDiff = np.abs(dot1[1] - dot2[1])
            distanceFromCenterDiff = np.abs(dot1[2] - dot2[2])
            dot1X, dot1Y = calXY(dot1[1], dot1[2])
            dot2X, dot2Y = calXY(dot2[1], dot2[2])
            distanceDiff = np.sqrt((dot1X - dot2X) ** 2 + (dot1Y - dot2Y) ** 2)
            info.append((angleDiff, distanceDiff, distanceFromCenterDiff))


def calXY(angle, dist):
    x = dist * np.cos(np.radians(angle))
    y = dist * np.sin(np.radians(angle))
    return x, y
