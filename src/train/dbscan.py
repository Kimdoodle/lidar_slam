import os
import pandas as pd
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import numpy as np
import time

current_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

def dbscan(csvPath: str, csvSavePath: str, pngSavePath: str, threshold: int, point_size: int):
    minPts = 4
    eps = None

    # 결과 저장 디렉토리 생성
    os.makedirs(csvSavePath, exist_ok=True)
    os.makedirs(pngSavePath, exist_ok=True)

    # SavePath 내의 모든 파일 삭제
    for file in os.listdir(csvSavePath):
        file_path = os.path.join(csvSavePath, file)
        if os.path.isfile(file_path):
            os.remove(file_path)
    for file in os.listdir(pngSavePath):
        file_path = os.path.join(pngSavePath, file)
        if os.path.isfile(file_path):
            os.remove(file_path)

    print("클러스터링 시작.")
    time.sleep(1)

    # 디렉토리 내의 모든 CSV 파일을 순회
    for filename in os.listdir(csvPath):
        if filename.endswith('.csv'):
            filepath = os.path.join(csvPath, filename)
            data = pd.read_csv(filepath).iloc[:, :-1]

            # 데이터가 100개 미만인 경우 클러스터링을 하지 않음
            if len(data) < 100:
                continue

            # interInfoLeft 속성 분포도의 x%에 해당하는 값을 eps로 설정
            eps = np.percentile(data['InterInfoLeft'], threshold)

            # DBSCAN 실행
            db = DBSCAN(eps=eps, min_samples=minPts).fit(data)
            labels = db.labels_

            # 클러스터링 결과를 새로운 컬럼에 저장
            data['Cluster'] = labels

            # csv 저장
            csv_save_path = os.path.join(csvSavePath, filename)
            data.to_csv(csv_save_path)
            print(f"CSV saved to {csv_save_path}")

            # 시각화
            plt.figure(figsize=(10, 6))
            plt.scatter(data['x'][labels == -1], data['y'][labels == -1], c='red', s=point_size, label='Outlier')
            plt.scatter(data['x'][labels != -1], data['y'][labels != -1], c='blue', s=point_size, label='Clustered Data')
            plt.title(f'Clusters and Outliers in {filename}')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()

            # 그림 파일로 저장
            plot_path = os.path.join(pngSavePath, filename.replace('.csv', '.png'))
            plt.savefig(plot_path)
            plt.close()
            print(f"Plot saved to {plot_path}")

if __name__ == '__main__':
    srcPath = os.path.join(current_dir, 'log', '2024-05-18(0)', 'convert')
    csvPath = os.path.join(current_dir, 'src', 'train', 'csvResult')
    pngPath = os.path.join(current_dir, 'src', 'train', 'pngResult')
    dbscan(srcPath, csvPath, pngPath, 99, point_size=5)
