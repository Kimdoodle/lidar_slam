import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from src.scanLog import scandata
import numpy as np
import pandas as pd

# 기존 텍스트 파일에서 학습 CSV 파일 생성
def convertCSV(filepath: str):
    l = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()[1:-1]
            # 데이터 포맷에 따라 적절히 조정
            data = line.split(', ')
            # 스캔 데이터에 맞는 형태로 변환 (예: 각도와 거리가 필요할 경우)
            l.append([int(data[0]), float(data[1]), float(data[2])])
    
    scan = scandata.Scan(l)
    scan.postProcess()  # 학습 데이터 생성

    # pandas DataFrame으로 데이터 구성
    df = pd.DataFrame({
        'x': scan.x,
        'y': scan.y,
        'InterInfoLeft': scan.interInfoLeft,
        'InterInfoRight': scan.interInfoRight,
        'AngleInfoLeft': scan.angleInfoLeft,
        'AngleInfoRight': scan.angleInfoRight,
        'DistInfoLeft': scan.distInfoLeft,
        'DistInfoRight': scan.distInfoRight
    })

    # 저장할 폴더 경로 설정
    data_folder = './src/train/data'
    # 폴더가 존재하지 않는 경우 생성
    os.makedirs(data_folder, exist_ok=True)
    
    # 출력 파일 경로 설정 (원본 파일 이름 사용)
    output_path = os.path.join(data_folder, os.path.basename(filepath))
    # 확장자를 .csv로 변경
    output_path = os.path.splitext(output_path)[0] + '.csv'
    
    # CSV 파일로 저장
    df.to_csv(output_path, index=False)
    print(f"Data saved to {output_path}")
