import os

import numpy as np

file_path = os.path.abspath(__file__)
src_path = os.path.abspath(os.path.join(file_path, '..', '..'))
project_path = os.path.abspath(os.path.join(src_path, '..'))
log_path = os.path.join(project_path, 'log')

map_path = os.path.join(log_path, 'map')
map_path1 = os.path.join(map_path, 'origmap.pgm')
map_path2 = os.path.join(map_path, 'mymap.pgm')

def load_pgm(file_path):
    with open(file_path, 'r') as f:
        first_line = f.readline().strip()
        print(f"First line of {file_path}: {first_line}")
        assert first_line == 'P2', f"First line is not P2 but {first_line}"  # PGM 파일의 매직 넘버
        f.readline()  # 주석 줄
        width, height = map(int, f.readline().strip().split())
        max_val = int(f.readline().strip())
        data = []
        for line in f:
            data.extend(map(int, line.strip().split()))
        return np.array(data).reshape((height, width))

def compare_pgm(file1, file2):
    map1 = load_pgm(file1)
    map2 = load_pgm(file2)
    difference = np.sum(map1 != map2)
    print(f"Number of different cells: {difference}")

# 사용 예제
print(f"Comparing {map_path1} and {map_path2}")
compare_pgm(map_path1, map_path2)
