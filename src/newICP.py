import numpy as np
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import cKDTree


def icp(A: list, B: list):
    # 1. subsampling진행 - 길이가 작은 쪽이 기준이 됨.
    min = A[:] if len(A) < len(B) else B[:]
    max = A[:] if min == B else B[:]

    # 조합 검색
    newMax = [-99 for i in range(len(max))]
    while(-99 in newMax):
        indexes = cKDTree(max).query(min, k=1)[1]
        # min의 점 중에서
        pass






    # 조합에 없는 데이터는 배제

    max = [max[index] for index in newMax]

    assert len(min) == len(max)
    print(min)
    print(max)

if __name__ == '__main__':
    A = [(0, 0), (3, 0.1)]
    B = [(0, 0), (0, -3), (-99, -99)]
    icp(A, B)

