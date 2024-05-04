try:
    import fcntl

    import serial

    from scanLog.rplidar import RPLidar
except: pass
import os
import time
import tkinter

import graphics
import mapdata
import scandata
import scanLog

mapData = mapdata.Map()

def is_serial_open(port):
    try:
        # 시리얼 포트를 열어보고 열려있는지 확인
        with serial.Serial(port) as ser:
            return ser.is_open
    except serial.SerialException:
        return False

def force_serial_close(port):
    try:
        # 시리얼 포트 강제 종료
        with open(port, 'r+') as ser_port:
            fcntl.flock(ser_port, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.system(f"stty -F {port} -hupcl")
    except Exception as e:
        print(f"Error: {e}")

def update_lidar_data():
    for i, scan in enumerate(lidar.iter_scans()):
        data = scandata.Scan(scan, i)
        # scanLog.saveScanLog(data)  # 로그 데이터 저장
        mapData.update(data)

        app.draw_lidar_data(mapData)
        
        root.update_idletasks()
        root.update()

def debug_lidar_data():
    logData = scanLog.loadScanLog("./src/log2")
    mapData = mapdata.Map()
    for index, log in enumerate(logData):
        data = scandata.Scan(log, index)
        mapData.update(data)
        app.draw_lidar_data(mapData)
        root.update_idletasks()
        root.update()
        time.sleep(5)

def on_closing():
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except: pass
    finally:
        root.destroy()

if __name__ == '__main__':
    root = tkinter.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    try:
        serial_port = '/dev/ttyUSB0'

        # if is_serial_open(serial_port):
        #     print(f"Serial port {serial_port} is open. Forcing close...")
        #     force_serial_close(serial_port)

        lidar = RPLidar(serial_port, 256000)

        info = lidar.get_info()
        #print(info)

        health = lidar.get_health()
        #print(health)

        app = graphics.LidarVisualization(root)

        root.after(5001, update_lidar_data)

        root.mainloop()

    except Exception as e:
        #print(e)
        # 디버깅 모드
        app = graphics.LidarVisualization(root)
        root.after(1, debug_lidar_data)

        root.mainloop()
        
        