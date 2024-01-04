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