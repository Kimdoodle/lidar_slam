# 로그 데이터 스캔
import os
import re


# 로그 파일을 스캔하여 좌표 데이터 반환
def load(path: str) -> list:
    try:
        # 파일 목록을 구성
        fileList = os.listdir(path)

        # 파일 데이터 반환
        fileData = []
        for file in fileList:
            text = open(path + '/' + file, 'r').read()
            data = re.findall(r'\((.*?)\)', text)

            # tuple 형태로 저장
            data_list = []
            for item in data:
                ele = tuple(map(float, item.split(',')))
                # 근거리 데이터는 장애물로 인식하여 포함하지 않음
                if ele[2] > 500:
                    data_list.append(ele[1:])
            fileData.append(data_list)

        return fileData
    except Exception as e:
        print(e)
