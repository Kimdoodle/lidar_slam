import os

import cv2
import numpy as np
import pandas as pd
from skimage.metrics import structural_similarity as ssim

# 경로 설정
file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')
map_path = os.path.join(log_path, 'map')

# PGM 파일 리스트
pgm_files = [os.path.join(map_path, f'np{i}.pgm') for i in range(1, 6)]

def read_custom_pgm(file_path):
    with open(file_path, 'rb') as f:
        header = f.readline().decode().strip()
        if header != 'P5':
            raise ValueError("Not a binary PGM file")
        
        # Read the rest of the header
        dimensions = f.readline().decode().strip()
        width, height = map(int, dimensions.split())
        max_value = int(f.readline().decode().strip())

        # Read the binary image data
        image_data = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))
        
        # OccupancyGrid와 동일한 변환 (-1: unknown, 0: free, 100: occupied)
        image_data = np.where(image_data == 205, -1, image_data)  # 중간 값 -> unknown
        image_data = np.where(image_data == 255, 0, image_data)   # white -> free
        image_data = np.where(image_data == 0, 100, image_data)   # black -> occupied

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
