import tkinter as tk
from tkinter import ttk
from math import cos, sin, radians
import random
import threading

DISTANCE_RATIO = 3000

class LidarVisualization:
    def __init__(self, root):
        self.root = root
        self.root.title("MAP")

        self.loading_frame = tk.Frame(root)
        self.loading_frame.pack(pady=10)

        self.loading_bar = ttk.Progressbar(self.loading_frame, maximum=100, mode='determinate', length=200)
        self.loading_bar.grid(row=0, column=0, padx=10)

        self.canvas = tk.Canvas(root, width=root.winfo_screenwidth(), height=root.winfo_screenheight(), bg="white")
        self.canvas.pack()

    # 로딩바 시작
    def start_loading_bar(self):
        self.loading_bar.start()

    # 로딩바 멈춤
    def stop_loading_bar(self):
        self.loading_bar.stop()
        self.loading_frame.grid_remove()
        self.draw_lidar_data([])


    # 스캔 데이터를 지도에 표시
    def draw_lidar_data(self, lidar_data):
        self.canvas.delete("all")

        max_distance = 300  # 최대 거리
        radius = 5  # 점 하나의 반지름
        # 중심점 계산
        center_x = self.canvas.winfo_reqwidth() / 2
        center_y = self.canvas.winfo_reqheight() / 2

        for _ , angle, distance in lidar_data:
            # print(f'angle: {angle}, distance: {distance}\n')

            # 좌표 계산
            x = center_x + distance * center_x * sin(radians(angle)) / DISTANCE_RATIO
            y = center_y - distance * center_y * cos(radians(angle)) / DISTANCE_RATIO

            # 각 데이터 
            self.canvas.create_rectangle(x - radius, y - radius, x + radius, y + radius, fill="blue")

        # 중심점
        self.canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, fill="red")


if __name__ == '__main__':
    root = tk.Tk()

    app = LidarVisualization(root)
    app.start_loading_bar()
    root.after(5000, app.stop_loading_bar)

    root.mainloop()

    # debug - for graphics test
    def update_data():
        if root.winfo_exists():
            data = []
            for _ in range(100):
                quality = random.randint(0, 100)
                angle = random.uniform(0, 360)
                distance = 300

                data.append((quality, angle, distance))
            print(data)
            app.draw_lidar_data(data)
        threading.Timer(0.5, update_data).start()

    update_data()



    







