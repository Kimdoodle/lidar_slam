# FOR DEBUG - Logfile save/load
import datetime
import os
import pandas as pd
import scandata
import signal
import sys
import threading
import traceback
import rospy

# 경로 추가
current_path = os.path.dirname(__file__)
sys.path.append(os.path.abspath(os.path.join(current_path, '..', 'lidar')))

from subscriber import LidarSubscriber

# 폴더 생성
def make_folders() -> tuple:
    folder_count = 0
    current_path = os.path.dirname(__file__)
    project_path = os.path.abspath(os.path.join(current_path, '..', '..'))
    cur_time = datetime.datetime.now().strftime("%Y-%m-%d") + f"({folder_count})"
    log_dir = os.path.join(project_path, 'log', cur_time)
    while os.path.exists(log_dir):
        folder_count += 1
        cur_time = datetime.datetime.now().strftime("%Y-%m-%d") + f"({folder_count})"
        log_dir = os.path.join(project_path, 'log', cur_time)

    raw_path = os.path.join(log_dir, 'raw')
    conv_path = os.path.join(log_dir, 'convert')
    os.makedirs(raw_path)
    os.makedirs(conv_path)

    return raw_path, conv_path

# 로그 스캔
def save_scan_log(lidar_subscriber, shutdown_flag):
    raw_path, conv_path = make_folders()
    count = 0

    try:
        same_sec_timestamp = 0
        current_time = ''
        while not rospy.is_shutdown() and not shutdown_flag.is_set():
            lidar_subscriber.wait_for_data()
            scan_data = lidar_subscriber.get_scan_data()
            next_time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            if current_time != next_time:
                same_sec_timestamp = 0
                current_time = next_time
            else:
                same_sec_timestamp += 1
            filename = f'log_{next_time}({same_sec_timestamp}).csv'
            file_directory = os.path.join(raw_path, filename)

            # DataFrame으로 변환
            df = pd.DataFrame(scan_data, columns=['angle', 'distance'])

            # CSV파일로 저장
            df.to_csv(file_directory, index=False)

            print(f"Scan data log successfully saved to {filename}, data: {len(scan_data)}")
            count += 1

    except Exception as e:
        traceback.print_exc()
        print("데이터 수: ", count)
    finally:
        print("학습 데이터 생성 시작.")
        make_train_data(raw_path, conv_path)

# 스캔 데이터를 학습 데이터로 저장
def make_train_data(raw_path: str, conv_path: str):
    count = 0
    file_list = sorted(os.listdir(raw_path))
    for file in file_list:
        temp = []  # 각 파일을 처리할 때마다 temp를 초기화합니다.
        df = pd.read_csv(os.path.join(raw_path, file))
        for _, row in df.iterrows():
            temp.append((0, row['angle'], row['distance']))  # quality 값을 0으로 설정

        scan = scandata.Scan(temp, convert=False)
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
        output_path = os.path.join(conv_path, f"{base_filename}_train.csv")

        # CSV 파일로 저장
        df.to_csv(output_path, index=False)
        print(f"Train data saved to {output_path}, data: {len(df)}")  # 로그 추가
        count += 1

    print(f"{count} train data saved.")


def load_scan_log(path="/log") -> tuple:
    try:
        # 파일 목록을 구성
        base = os.path.dirname(os.path.abspath(__file__))
        path = base + path
        file_list = sorted(os.listdir(path))
        
        # 파일 데이터 반환
        file_data = []
        for file in file_list:
            df = pd.read_csv(os.path.join(path, file))

            # tuple List 형태로 저장
            data_list = []
            for _, row in df.iterrows():
                ele = (0, row['angle'], row['distance'])  # 임시로 quality를 0으로 설정
                data_list.append(ele)
            file_data.append(data_list)

        return file_list, file_data
    except Exception as e:
        print(e)

if __name__ == '__main__':
    lidar_subscriber = LidarSubscriber()
    shutdown_flag = threading.Event()

    scan_thread = threading.Thread(target=save_scan_log, args=(lidar_subscriber, shutdown_flag))
    scan_thread.start()

    input("Press Enter to stop...\n")
    shutdown_flag.set()

    scan_thread.join()
    print("Scan log has been saved and training data created.")
