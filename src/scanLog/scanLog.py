# FOR DEBUG - Logfile save/load
import datetime
import os
import re
import time
import traceback
import pandas as pd
import scandata

try:
    import fcntl
    import serial

    from rplidar import RPLidar
except:
    pass


# 로그 스캔, 1초마다 반복함
def saveScanLog():
    current_dir = os.path.join(os.getcwd(), 'raw')
    curTime = 'log' + datetime.datetime.now().strftime("%Y-%m-%d")
    
    os.makedirs(os.path.join(current_dir, curTime), exist_ok=True)

    try:
        # Lidar 실행
        serial_port = '/dev/ttyUSB0'
        lidar = RPLidar(serial_port, 256000)
        time.sleep(6)

        # 스캔
        sameSecTimestamp = 0
        currentTime = ''
        for i, scan in enumerate(lidar.iter_scans()):
            data = scandata.Scan(scan)
            nextTime = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            if currentTime != nextTime:
                sameSecTimestamp = 0
                currentTime = nextTime
            else:
                sameSecTimestamp += 1
            filename = f'log_{nextTime}({sameSecTimestamp}).csv'
            filedirectory = os.path.join(current_dir, curTime, filename)

            # DataFrame으로 변환
            df = pd.DataFrame(data.scanInfo, columns=['quality', 'angle', 'distance'])

            # CSV파일로 저장
            df.to_csv(filedirectory, index=False)

            print(f"Scan data log successfully saved to {filename}, data: {len(data.scanInfo)}")

            # time.sleep(1)
    except Exception as e:
        # print(f"Error saving scan data log: {e}")
        traceback.print_exc()
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

# return [logFileNames, logData]
def loadScanLog(path="/log") -> tuple:
    try:
        # 파일 목록을 구성
        base = os.path.dirname(os.path.abspath(__file__))
        path = base + path
        fileList = sorted(os.listdir(path))
        
        # 파일 데이터 반환
        fileData = []
        for file in fileList:
            df = pd.read_csv(os.path.join(path, file))

            # tuple List 형태로 저장
            data_list = []
            for _, row in df.iterrows():
                ele = (row['quality'], row['angle'], row['distance'])
                data_list.append(ele)
            fileData.append(data_list)

        return fileList, fileData
    except Exception as e:
        print(e)

# def saveCalLog(scan):
#     try:
#         filename = f'calLog.txt'
#         filedirectory = os.path.join(os.path.dirname(os.path.realpath(__file__)), filename)
#
#         # 파일을 현재 디렉토리에 저장
#         with open(filedirectory, 'w') as file:
#             file.write('cordInfo(quality, angle, distance, x, y)\n')
#             for index, cord in enumerate(scan.cordInfo):
#                 file.write(str(index) + '\t' + cord.toString() + '\n')
#             file.write('angleInfo(diff between i, i+1)\n')
#             file.write(f'평균: {scan.angleMean}, 표준편차: {scan.angleStd}\n')
#             for index, angle in enumerate(scan.angleInfo):
#                 file.write(str(index) + '\t' + str(angle) + '\n')
#             file.write('distInfo(diff between i, i+1)\n')
#             file.write(f'평균: {scan.distMean}, 표준편차: {scan.distStd}\n')
#             for index, dist in enumerate(scan.distInfo):
#                 file.write(str(index) + '\t' + str(dist) + '\n')
#             file.write('interInfo(diff between to dots)\n')
#             file.write(f'평균: {scan.interMean}, 표준편차: {scan.interStd}\n')
#             for index, inter in enumerate(scan.interInfo):
#                 file.write(str(index) + '\t' + str(inter) + '\n')
#             file.write('funcInfo(function)\n')
#             file.write(f'평균: {scan.funcMean}, 표준편차: {scan.funcStd}\n')
#             for index, func in enumerate(scan.funcInfo):
#                 file.write(str(index) + '\t' + str(func) + '\n')
#             file.write('lineInfo(start, end)\n')
#             for index, line in enumerate(scan.lineInfo):
#                 file.write(str(index) + '\t' + line.toString() + '\n')
#
#         print(f"Scan data log successfully saved to {filename}")
#     except Exception as e:
#         print(f"Error saving scan data log: {e}")


if __name__ == '__main__':
    saveScanLog()