# 두 스캔 데이터를 병합
import numpy as np
from icp import nearest_neighbor
from scandata import Scan
from scipy.spatial import cKDTree

'''
    각각의 스캔 데이터를 직선별로 분리
    서로 가장 가까운 직선을 검색함
    후직선을 선직선의 기울기에 맞추어 병합
'''

# def merge(data1:list, data2:list) -> list:
#     # 개수 조절
#     len1 = len(data1)
#     len2 = len(data2)
#     if len1 > len2:
#         data1 = align_datasets(data1, data2)
#     elif len2 > len1:
#         data2 = align_datasets(data2, data1)
#     else: pass

#     i = -100
#     result = (-999, 9999)

#     while i <= 100:
#         temp = compare(data1, data2, i)
#         print(f'result: {str((i, temp))}')
#         if result[1] > temp:
#             result = (i, temp)
#         i += 1
#     print(f'final result: {str(result)}')
    

# # data2의 좌표를 일괄 x만큼 이동한 후 두 데이터를 비교
# def compare(data1, data2, x):
#     # 이동
#     cord1 = np.array(data1)
#     cord2 = np.array([(a[0]+x, a[1], 0) for a in data2])

#     beforeTree = cKDTree(cord1)
#     distances, indices = beforeTree.query(cord2)

#     mean, std = removeOutlier(distances)

#     return mean

# # 배열의 개수에 맞추어 가장 튀는 점을 제거
# def align_datasets(large, small):
#     tree = cKDTree(small)

#     # 가장 가까운 이웃 찾기
#     distances, indices = tree.query(large, k=len(small))

#     # 거리가 큰 데이터부터 삭제하여 길이 맞추기
#     dist = [min(element) for element in distances]
#     remove = len(large) - len(small)
#     for i in range(remove):
#         index = dist.index(max(dist))
#         dist.pop(index)
#         large.pop(index)

#     return large

# # 이상치 제거 후 평균, 표준편차 계산
# def removeOutlier(data:list):
    # mean = np.mean(np.array(data))
    # std = np.std(np.array(data))

    # newData = []
    # for d in data:
    #     zScore = (d-mean)/std
    #     if np.abs(zScore) <= 1:
    #         newData.append(d)
    
    # newMean = np.mean(np.array(newData))
    # newStd = np.std(np.array(newData))
    # return newMean, newStd