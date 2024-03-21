import numpy as np
from sklearn.neighbors import NearestNeighbors

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

# move를 변형하여 orig에 맞춘다
def icp_with_subsample(A, B, max_iterations=20, tolerance=0.001):

    assert A.shape[1] == B.shape[1]

    m = A.shape[1]  # 차원 수

    src = np.ones((m + 1, A.shape[0]))
    dst = np.ones((m + 1, B.shape[0]))
    src[:m, :] = np.copy(A.T)
    dst[:m, :] = np.copy(B.T)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T)

        # subsample max_cloud based on the nearest neighbors in min_cloud
        subsampled_dst = dst[:m, indices].T

        # compute the transformation between the subsampled source and destination points
        T, _, _ = best_fit_transform(src[:m, :].T, subsampled_dst)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T, _, _ = best_fit_transform(min_cloud, src[:m, :].T)

    return T, distances, i

def nearest_neighbor(src, dst):
    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()



if __name__ == '__main__':
    # 예시
    A = np.array([[0, 0], [3, 0.1]])
    B = np.array([[0, 0], [0, -3]])

    # max, min 구분
    max_cloud = A if len(A) >= len(B) else B
    min_cloud = B if len(A) >= len(B) else A

    # ICP 적용
    T, distances, iterations = icp_with_subsample(min_cloud, max_cloud)

    print("Transformation Matrix:")
    print(T)
    print("Euclidean Distances:")
    print(distances)
    print("Number of Iterations:", iterations)
