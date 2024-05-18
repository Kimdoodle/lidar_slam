import sys
import gui
import scanFile
import os

def main():
    current_path = os.path.dirname(os.path.abspath(__file__))
    dirPath = os.path.abspath(os.path.join(current_path, '..', '..'))
    path = os.path.join(dirPath, 'log', '2024-05-18(1)', 'convert')

    # 경로 디버깅 출력
    print(f"Current path: {current_path}")
    print(f"dirPath: {dirPath}")
    print(f"Target path: {path}")

    # 데이터 로드 및 처리
    gui.graphics_odometry([[0, 0]])

    data_generator = scanFile.load(path)
    
    try:
        if data_generator is not None:
            for batch_data in data_generator:
                if batch_data:
                    gui.graphics_cord(batch_data, 1)
        else:
            print(f"No data found in path: {path}")
    except KeyboardInterrupt as e:
        sys.exit()

if __name__ == "__main__":
    main()
