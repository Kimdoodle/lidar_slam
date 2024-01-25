my_list = [5, 3, 8, 2, 1, 7, 4, 6]
n = 3  # 추출할 원소의 개수

# 가장 작은 순서부터 n개의 원소 추출
min_indices = sorted(range(len(my_list)), key=lambda k: my_list[k])[:n]

print("가장 작은 순서부터 {}개의 원소의 인덱스: {}".format(n, min_indices))
