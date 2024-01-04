from rplidar import RPLidar
import scanLog
import tkinter
import graphics
import time
import serial
import sys
import os
import fcntl

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
        scanLog.saveLog(scan)  # 로그 데이터 저장
        app.draw_lidar_data(scan)
        root.update_idletasks()
        root.update()


def on_closing():
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    root.destroy()

if __name__ == '__main__':
    try:
        serial_port = '/dev/ttyUSB0'

        if is_serial_open(serial_port):
            print(f"Serial port {serial_port} is open. Forcing close...")
            force_serial_close(serial_port)

        lidar = RPLidar(serial_port, 256000)
        root = tkinter.Tk()
        root.protocol("WM_DELETE_WINDOW", on_closing)
        info = lidar.get_info()
        print(info) 

        health = lidar.get_health()
        print(health)

        app = graphics.LidarVisualization(root)
        app.start_loading_bar()

        root.after(5000, app.stop_loading_bar)
        root.after(5001, update_lidar_data)

        root.mainloop()

    except Exception as e:
        print(f'Error! {e}')
        on_closing()