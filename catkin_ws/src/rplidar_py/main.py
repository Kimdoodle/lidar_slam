from rplidar import RPLidar

# 연결
def test():
    while True:
        try:
            lidar = RPLidar('/dev/ttyUSB0', 256000)
            info = lidar.get_info()
            print(info)

            health = lidar.get_health()
            print("Health" + str(health))

            print('connect success')

            for i, scan in enumerate(lidar.iter_scans()):
                print('%d: Got %d measurments' % (i, len(scan)))
                for element in scan:
                    print(str(scan) + '\n')
                if i > 10:
                    break
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            return
        except:
            pass



if __name__ == '__main__':
    test()

