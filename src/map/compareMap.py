import os
import numpy as np
import pandas as pd
from skimage.metrics import structural_similarity as ssim
import cv2

# 경로 설정
file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')
map_path = os.path.join(log_path, 'map')

# PGM 파일 리스트
pgm_files = [os.path.join(map_path, f'final_p{i}.pgm') for i in range(1, 6)]

def read_custom_pgm(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    # 데이터만 추출 (공백을 기준으로 분리)
    data = []
    for line in lines:
        data.extend([int(x) for x in line.split()])
    # OccupancyGrid와 동일한 변환
    width = int(np.sqrt(len(data)))  # 정사각형 형태 가정
    height = width
    image_data = np.array(data).reshape((height, width))
    image_data = np.where(image_data == -1, 205, image_data)  # unknown -> 205 (중간 값)
    image_data = np.where(image_data == 0, 255, image_data)   # free -> 255 (white)
    image_data = np.where(image_data == 100, 0, image_data)  # occupied -> 0 (black)
    return image_data.astype(np.uint8)

# MSE 계산 함수
def calculate_mse(imageA, imageB):
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])
    return err

# PSNR 계산 함수
def calculate_psnr(imageA, imageB):
    mse = calculate_mse(imageA, imageB)
    if mse == 0:
        return 100
    PIXEL_MAX = 255.0
    return 20 * np.log10(PIXEL_MAX / np.sqrt(mse))

# 파일 경로 및 이미지 확인
images = []
for file in pgm_files:
    if not os.path.isfile(file):
        print(f"File {file} does not exist")
    else:
        try:
            image = read_custom_pgm(file)
            images.append(image)
            print(f"Successfully loaded {file}")
        except Exception as e:
            print(f"Error reading {file}: {e}")

# 결과 저장
results = []

# 모든 파일 쌍에 대해 정확도 비교
for i in range(len(images)):
    for j in range(i + 1, len(images)):
        # 이미지 읽기
        image1 = images[i]
        image2 = images[j]

        if image1 is None or image2 is None:
            print(f"Skipping comparison between {pgm_files[i]} and {pgm_files[j]} due to loading issue.")
            continue

        # MSE, PSNR, SSIM 계산
        mse_value = calculate_mse(image1, image2)
        psnr_value = calculate_psnr(image1, image2)
        ssim_value, _ = ssim(image1, image2, full=True)

        # 결과 저장
        results.append({
            'File1': os.path.basename(pgm_files[i]),
            'File2': os.path.basename(pgm_files[j]),
            'MSE': mse_value,
            'PSNR': psnr_value,
            'SSIM': ssim_value
        })


# 결과 출력
for result in results:
    print(f"Comparing {result['File1']} and {result['File2']}:")
    print(f"  MSE: {result['MSE']}")
    print(f"  PSNR: {result['PSNR']}")
    print(f"  SSIM: {result['SSIM']}\n")


# 결과를 데이터프레임으로 저장
df = pd.DataFrame(results)
csv_path = os.path.join(map_path, 'pgm_comparison_results.csv')
df.to_csv(csv_path, index=False)
print(f"Results saved to {csv_path}")
