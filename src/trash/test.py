import pandas as pd

# 파일 경로 설정
file_path = '/home/abcd/catkin_ws/src/LiDAR_SLAM/log/2024-05-18(0)/convert/log_2024-05-18-17-48-23(0)_train.csv'

# CSV 파일 읽기
df = pd.read_csv(file_path)

# 각도 범위 계산
min_angle = df['AngleInfoLeft'].min()
max_angle = df['AngleInfoRight'].max()

# 각도 범위 출력
print(f"Angle range: {min_angle} to {max_angle}")
