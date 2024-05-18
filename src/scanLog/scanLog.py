# FOR DEBUG - Logfile save/load
import datetime
import os
import time
import traceback
import pandas as pd
import scandata
import signal
import sys
import shutil

try:
    import fcntl
    import serial

    from rplidar import RPLidar
except:
    pass

# 폴더 생성
def makeFolders() -> tuple:
    folderCount = 0
    current_path = os.path.dirname(__file__)
    project_path = os.path.abspath(os.path.join(current_path, '..', '..'))
    curTime = datetime.datetime.now().strftime("%Y-%m-%d") + f"({folderCount})"
    logDir = os.path.join(project_path, 'log', curTime)
    while os.path.exists(logDir):
        folderCount += 1
        curTime = datetime.datetime.now().strftime("%Y-%m-%d") + f"({folderCount})"
        logDir = os.path.join(project_path, 'log', curTime)

    rawPath = os.path.join(logDir, 'raw')
    convPath = os.path.join(logDir, 'convert')
    os.makedirs(rawPath)
    os.makedirs(convPath)

    return rawPath, convPath

# 로그 스캔, 1초마다 반복함
def saveScanLog():
    rawPath, convPath = makeFolders()
    lidar = None
    count = 0
    def signal_handler(sig, frame):
        if lidar:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        print("학습 데이터 생성 시작.")
        makeTrainData(rawPath, convPath)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Lidar 실행
        serial_port = '/dev/ttyUSB0'
        lidar = RPLidar(serial_port, 256000)
        time.sleep(6)
        # # 작동 확인
        # for i, scan in enumerate(lidar.iter_scans()):
        #     try:
        #         data = scandata.Scan(scan, convert=False)
        #     except:
        #         time.sleep(1)
                
        # 스캔
        sameSecTimestamp = 0
        currentTime = ''
        for i, scan in enumerate(lidar.iter_scans()):
            data = scandata.Scan(scan, convert=False)
            nextTime = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            if currentTime != nextTime:
                sameSecTimestamp = 0
                currentTime = nextTime
            else:
                sameSecTimestamp += 1
            filename = f'log_{nextTime}({sameSecTimestamp}).csv'
            filedirectory = os.path.join(rawPath, filename)

            # DataFrame으로 변환
            df = pd.DataFrame(data.scanInfo, columns=['quality', 'angle', 'distance'])

            # CSV파일로 저장
            df.to_csv(filedirectory, index=False)

            print(f"Scan data log successfully saved to {filename}, data: {len(data.scanInfo)}")
            count += 1

            # time.sleep(1)
    except Exception as e:
        # print(f"Error saving scan data log: {e}")
        traceback.print_exc()
        print("데이터 수: ", count)
    except KeyboardInterrupt as e:
        print("키보드 종료 입력감지, 학습 데이터 생성 시작.")
        makeTrainData(rawPath, convPath)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    finally: # 스캔 종료와 동시에 데이터 변환, 저장
        print("학습 데이터 생성 시작.")
        makeTrainData(rawPath, convPath)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

# 스캔 데이터를 학습 데이터로 저장
def makeTrainData(rawPath: str, convPath: str):
    temp = []
    count = 0
    fileList = sorted(os.listdir(rawPath))
    for file in fileList:
        df = pd.read_csv(os.path.join(rawPath, file))
        for _, row in df.iterrows():
            temp.append((row['quality'], row['angle'], row['distance']))

        scan = scandata.Scan(temp)
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
            'DistInfoRight': scan.distInfoRight,
            'class': 0
        })
        # 출력 파일 경로 설정 (원본 파일 이름 사용, 확장자를 .csv로 변경)
        base_filename = os.path.splitext(file)[0]
        output_path = os.path.join(convPath, f"{base_filename}_train.csv")

        # CSV 파일로 저장
        df.to_csv(output_path, index=False)
        count += 1

    print(f"{count} train data saved.")


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


if __name__ == '__main__':
    saveScanLog()