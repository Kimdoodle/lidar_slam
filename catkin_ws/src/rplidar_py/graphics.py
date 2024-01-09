import random
import threading
import tkinter as tk
from math import cos, radians, sin
from tkinter import ttk

import mapdata

MOVE_RATIO = 50

class LidarVisualization:
    def __init__(self, root):
        self.mapData = None
        self.distance_ratio = 5

        self.root = root
        self.root.title("MAP")

        self.loading_frame = tk.Frame(root)
        self.loading_frame.pack(pady=10)

        self.loading_bar = ttk.Progressbar(self.loading_frame, maximum=100, mode='determinate', length=200)
        self.loading_bar.grid(row=0, column=0, padx=10)

        self.canvas = tk.Canvas(root, width=root.winfo_screenwidth(), height=root.winfo_screenheight(), bg="white")
        # 중심점 계산
        self.center_x = self.canvas.winfo_reqwidth() / 2
        self.center_y = self.canvas.winfo_reqheight() / 2
        self.canvas.bind("<Button-1>", self.increase_distance_ratio)
        self.canvas.bind("<Button-3>", self.decrease_distance_ratio)
        self.root.bind("<Up>", self.on_arrow_key)
        self.root.bind("<Down>", self.on_arrow_key)
        self.root.bind("<Left>", self.on_arrow_key)
        self.root.bind("<Right>", self.on_arrow_key)
        self.canvas.pack()

    # 마우스 좌클릭/우클릭으로 확대/축소 조절
    def increase_distance_rat절io(self, event):
        self.distance_ratio += 3
        print(self.distance_ratio)
        self.draw_lidar_data(None)

    def decrease_distance_ratio(self, event):
        self.distance_ratio -= 3
        if self.distance_ratio < 1:
            self.distance_ratio = 1
        print(self.distance_ratio)
        self.draw_lidar_data(None)

    # 방향키로 중심점 조절
    def on_arrow_key(self, event):
        key = event.keysym
        print(f'Key Pressed!, {key}')
        if key == 'Up':
            self.center_y += MOVE_RATIO
        elif key == 'Down':
            self.center_y -= MOVE_RATIO
        elif key == 'Left':
            self.center_x += MOVE_RATIO
        elif key == 'Right':
            self.center_x -= MOVE_RATIO

    # 로딩바 시작
    def start_loading_bar(self):
        self.loading_bar.start()

    # 로딩바 멈춤
    def stop_loading_bar(self):
        self.loading_bar.stop()
        self.loading_frame.grid_remove()
        self.draw_lidar_data([])

    # 스캔 데이터를 지도에 표시
    def draw_lidar_data(self, mapData):
        self.canvas.delete("all")
        if mapData == None:
            mapData = self.mapData
        else:
            self.mapData = mapData
        RADIUS = 5  # 점 하나의 반지름

        for lineInfo in mapData.lineInfo:
            x1 = self.center_x + (lineInfo.startX - RADIUS) / self.distance_ratio
            y1 = self.center_y + (lineInfo.startY - RADIUS) / self.distance_ratio
            x2 = self.center_x + (lineInfo.endX + RADIUS) / self.distance_ratio
            y2 = self.center_y + (lineInfo.endY + RADIUS) / self.distance_ratio
            self.canvas.create_line(x1, y1, x2, y2, width=2, fill='red')

        # # 점의 형태로 지도 생성
        # for index, cord in enumerate(mapData.cordInfo):
        #     x1 = center_x + (cord.x - radius) / self.distance_ratio
        #     y1 = center_y + (cord.y - radius) / self.distance_ratio
        #     x2 = center_x + (cord.x + radius) / self.distance_ratio
        #     y2 = center_y + (cord.y + radius) / self.distance_ratio

        #     self.canvas.create_rectangle(x1, y1, x2, y2, fill='black')
            
        #     # 중심에 번호 표시
        #     center_x_text = (x1 + x2) / 2
        #     center_y_text = (y1 + y2) / 2
        #     self.canvas.create_text(center_x_text, center_y_text, text=str(index))

        # 중심점
        self.canvas.create_oval(self.center_x - RADIUS, self.center_y - RADIUS, self.center_x + RADIUS, self.center_y + RADIUS, fill="red")

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