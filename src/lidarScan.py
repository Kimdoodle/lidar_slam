import threading
import time
import traceback

from rplidar import RPLidar


class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.lidar = RPLidar(port, baudrate)
        time.sleep(6)
        self.running = False
        self.scan_data = []
        self.lock = threading.Lock()

    def get_info(self):
        info = self.lidar.get_info()
        return '\n'.join('%s: %s' % (k, str(v)) for k, v in info.items())

    def get_health(self):
        return self.lidar.get_health()

    def _process_scan(self, scan):
        with self.lock:
            self.scan_data.append(scan)

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
                self._process_scan(scan)
                print(scan)
        except Exception as e:
            traceback.print_exc()
        finally:
            self.stop()

    def _stop_lidar(self):
        input("Press Enter to stop the LIDAR...\n")
        self.stop()

    def stop(self):
        if self.running:
            self.running = False
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("LIDAR stopped.")

    def get_scan_data(self):
        with self.lock:
            return list(self.scan_data)

if __name__ == '__main__':
    scanner = LidarScanner()
    print(scanner.get_info())
    print(scanner.get_health())
    scanner.start_scanning()
