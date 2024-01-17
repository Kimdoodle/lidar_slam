# FOR DEBUG - Logfile save/load
import datetime
import os
import re
import traceback


def saveScanLog(data):
    try:
        current_time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        filename = f'log_{current_time}.txt'
        filedirectory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'log', filename)

        # 파일을 현재 디렉토리에 저장
        with open(filedirectory, 'w') as file:
            for cord in data.cordInfo:
                text = '('+', '.join(map(str, cord.toLog())) + ')\n'
                file.write(text)

        print(f"Scan data log successfully saved to {filename}, data: {len(data.cordInfo)}")
    except Exception as e:
        #print(f"Error saving scan data log: {e}")
        traceback.print_exc()

# return [logFileNames, logData]
def loadScanLog(path="./src/log") -> list:
    try:
        # 파일 목록을 구성
        fileList = sorted(os.listdir(path))
        
        # 파일 데이터 반환
        fileData = []
        for file in fileList:
            text = open(path + '/' + file, 'r').read()
            data = re.findall(r'\((.*?)\)', text)

            # tuple List 형태로 저장
            data_list = []
            for item in data:
                ele = tuple(map(float, item.split(',')))
                if ele[2] > 500:
                    data_list.append(ele)
            # for data in data_list:
            #     if(data[0] < 15):
            #         print(data)
            fileData.append(data_list)

        return fileList, fileData
    except Exception as e:
        print(e)

def saveCalLog(scan):
    try:
        filename = f'calLog.txt'
        filedirectory = os.path.join(os.path.dirname(os.path.realpath(__file__)), filename)

        # 파일을 현재 디렉토리에 저장
        with open(filedirectory, 'w') as file:
            file.write('cordInfo(quality, angle, distance, x, y)\n')
            for index, cord in enumerate(scan.cordInfo):
                file.write(str(index) + '\t' + cord.toString() + '\n')
            file.write('angleInfo(diff between i, i+1)\n')
            file.write(f'평균: {scan.angleMean}, 표준편차: {scan.angleStd}\n')
            for index, angle in enumerate(scan.angleInfo):
                file.write(str(index) + '\t' + str(angle) + '\n')
            file.write('distInfo(diff between i, i+1)\n')
            file.write(f'평균: {scan.distMean}, 표준편차: {scan.distStd}\n')
            for index, dist in enumerate(scan.distInfo):
                file.write(str(index) + '\t' + str(dist) + '\n')
            file.write('interInfo(diff between to dots)\n')
            file.write(f'평균: {scan.interMean}, 표준편차: {scan.interStd}\n')
            for index, inter in enumerate(scan.interInfo):
                file.write(str(index) + '\t' + str(inter) + '\n')
            file.write('funcInfo(function)\n')
            file.write(f'평균: {scan.funcMean}, 표준편차: {scan.funcStd}\n')
            for index, func in enumerate(scan.funcInfo):
                file.write(str(index) + '\t' + str(func) + '\n')
            file.write('lineInfo(start, end)\n')
            for index, line in enumerate(scan.lineInfo):
                file.write(str(index) + '\t' + line.toString() + '\n')

        print(f"Scan data log successfully saved to {filename}")
    except Exception as e:
        print(f"Error saving scan data log: {e}")
