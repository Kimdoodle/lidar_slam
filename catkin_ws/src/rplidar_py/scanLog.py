import datetime
import os

def saveLog(data):
    try:
        current_time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'log_{current_time}.txt'
        filedirectory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'log', filename)

        # 파일을 현재 디렉토리에 저장
        with open(filedirectory, 'w') as file:
            for item in data:
                file.write(','.join(map(str, data)))

        print(f"Scan data log successfully saved to {filename}")
    except Exception as e:
        print(f"Error saving scan data log: {e}")


def loadLog():
    try:
        # 파일 목록을 구성
        path = "./catkin_ws/src/rplidar_py/log"
        fileList = os.listdir(path)
        
        # 파일 데이터 반환
        fileData = []
        for file in fileList:
            text = open(path + '/' + file, 'r').read()
            eleList = text.split("),(")
            eleList[0] = eleList[0][1:]
            eleList[-1] = eleList[-1][:-1]
            if eleList[-1][-1] == ')':
                eleList[-1] = eleList[-1][:-1]

            # tuple List 형태로 저장
            data_list = [tuple(map(float, item.split(','))) for item in eleList]
            fileData.append(data_list)

        return fileData
    except Exception as e:
        print(e)
        
if __name__ == '__main__':
    loadLog()
