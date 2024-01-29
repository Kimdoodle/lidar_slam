import numpy as np

def icp(A, B, max_iterations=100, tolerance=1e-6):
    # 초기 변환 행렬 설정 (이동 x축: -1, 이동 y축: 0, 회전: -90도)
    transformation = np.array([[0, -1, -1],
                               [1, 0, 0],
                               [0, 0, 1]])

    for iteration in range(max_iterations):
        # 현재 변환으로 포인트 클라우드 A를 변환
        A_transformed = np.dot(transformation[:2, :2], np.transpose(A)) + transformation[:2, 2:]

        # ICP 알고리즘의 주요 단계: 각 포인트에 대해 가장 가까운 이웃을 찾아 변환 업데이트
        distances, indices = find_nearest_neighbors(A_transformed, B)
        A_closest = B[indices]

        # 변환 행렬 업데이트
        new_transformation = compute_transformation(A_transformed, A_closest)

        # 변환 행렬이 수렴했는지 확인
        if np.linalg.norm(new_transformation - transformation) < tolerance:
            break

        # 변환 행렬 업데이트
        transformation = new_transformation

    return transformation

def find_nearest_neighbors(A, B):
    # A의 각 포인트에 대해 B에서 가장 가까운 이웃을 찾아 인덱스 반환
    distances = np.linalg.norm(A[:, np.newaxis, :] - B, axis=2)
    indices = np.argmin(distances, axis=1)
    return distances, indices

def compute_transformation(A, B):
    # 최소자승 문제를 푸는 방법으로 변환 행렬 업데이트
    H = np.dot(A, np.transpose(B))
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    t = np.mean(B, axis=0) - np.dot(R, np.mean(A, axis=0))
    transformation = np.eye(3)
    transformation[:2, :2] = R
    transformation[:2, 2] = t
    return transformation

# 포인트 클라우드 정의
A = np.array([(1, 0), (4, 0.1)])
B = np.array([(0, 0), (0, -3)])

# ICP 알고리즘 수행
result_transformation = icp(A, B)

# 변환 행렬을 A에 적용
A_transformed_result = np.dot(result_transformation[:2, :2], np.transpose(A)) + result_transformation[:2, 2:]

# 결과 출력
print("변환 행렬:")
print(result_transformation)
print("A를 변환한 결과:")
print(np.transpose(A_transformed_result))
