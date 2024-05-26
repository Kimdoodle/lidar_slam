import datetime
import os
import sys
import threading
import time
import traceback

import numpy as np
import pandas as pd
from rplidar import RPLidar

current_path = os.path.dirname(__file__)
sys.path.append(os.path.abspath(os.path.join(current_path, '..', 'scanLog')))

import scanLog

# make folders
raw_path, conv_path = scanLog.make_folders()
scanData = []

class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.lidar = RPLidar(port, baudrate)
        time.sleep(6)
        self.running = False

    def get_info(self):
        info = self.lidar.get_info()
        return '\n'.join('%s: %s' % (k, str(v)) for k, v in info.items())

    def get_health(self):
        return self.lidar.get_health()

    def start_scanning(self):
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan)
        self.scan_thread.start()
        self.stop_thread = threading.Thread(target=self._stop_lidar)
        self.stop_thread.start()

    def _scan(self):
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                current_time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
                timestamp = 0
                scan = np.array(scan)
                scan_data = np.array(scan[:, 1:])
                while True:
                    filename = f'log_{current_time}({timestamp}).csv'
                    filepath = os.path.join(raw_path, filename)
                    if os.path.isfile(filepath):
                        timestamp += 1
                    else:
                        break
                # Convert to DataFrame
                df = pd.DataFrame(scan_data, columns=['angle', 'distance'])

                # Save to CSV file
                df.to_csv(filepath, index=False)
                print(f"Scan data log successfully saved to {filename}, data: {len(scan_data)}")

        except Exception as e:
            traceback.print_exc()
            self.stop()

    def _stop_lidar(self):
        input("Press Enter to stop the LIDAR...\n")
        self.stop()

    def stop(self):
        self.running = False
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        print("LIDAR stopped.")
        print("Starting to create training data.")
        scanLog.make_train_data(raw_path, conv_path)

    def get_scan_data(self):
        return self.scan_data

if __name__ == "__main__":
    scanner = LidarScanner()
    print(scanner.get_info())
    print(scanner.get_health())
    scanner.start_scanning()
    
