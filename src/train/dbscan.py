import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import data
import pandas as pd
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import numpy as np
import time


# 모든 데이터를 csv data로 변환
def dataProcess(dirPath: str):
    # 폴더 내의 모든 파일을 순회
    for filename in os.listdir(dirPath):
        filepath = os.path.join(dirPath, filename)
        data.convertCSV(filepath)


def dbscan(dirPath: str, dirPath2: str):
    minPts = 4
    eps = None

    # 결과 저장 디렉토리 생성
    os.makedirs(dirPath2, exist_ok=True)

    print("클러스터링 시작.")
    time.sleep(1)

    # 디렉토리 내의 모든 CSV 파일을 순회
    for filename in os.listdir(dirPath):
        if filename.endswith('.csv'):
            filepath = os.path.join(dirPath, filename)
            data = pd.read_csv(filepath)

            # 데이터가 100개 미만인 경우 클러스터링을 하지 않음
            if len(data) < 100:
                continue

            # interInfoLeft 속성 분포도의 x%에 해당하는 값을 eps로 설정
            eps = np.percentile(data['InterInfoLeft'], 95)

            # DBSCAN 실행
            db = DBSCAN(eps=eps, min_samples=minPts).fit(data)
            labels = db.labels_

            # 클러스터링 결과를 새로운 컬럼에 저장
            data['Cluster'] = labels

            # 결과를 새 파일로 저장
            result_path = os.path.join(dirPath, filename.replace('.csv', '_clustered.csv'))
            data.to_csv(result_path, index=False)
            print(f"Clustering completed and saved to {result_path}")

            # 시각화
            plt.figure(figsize=(10, 6))
            plt.scatter(data['x'][labels == -1], data['y'][labels == -1], c='red', label='Outlier')
            plt.scatter(data['x'][labels != -1], data['y'][labels != -1], c='blue', label='Clustered Data')
            plt.title(f'Clusters and Outliers in {filename}')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()

            # 그림 파일로 저장
            plot_path = os.path.join(dirPath2, filename.replace('.csv', '.png'))
            print(plot_path)
            # plt.show()
            plt.savefig(plot_path)
            plt.close()
            print(f"Plot saved to {plot_path}")


if __name__ == '__main__':
    # dataProcess('./log/2024-05-05')
    dbscan('./src/train/data', './src/train/result')
