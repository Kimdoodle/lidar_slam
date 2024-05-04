from rplidar import RPLidar
import time

lidar = RPLidar('/dev/ttyUSB0', 256000)
time.sleep(6)
info =lidar.get_info()

print('\n'.join('%s: %s' % (k, str(v)) for k, v in info.items()))

lidar.get_health()

process_scan = lambda scan: None

try :
    for scan in lidar.iter_scans():
        print(scan)
except:
    lidar.stop()
    lidar.stop_motor()