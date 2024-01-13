# 로그 추출 및 디버깅용 파일
import re
import tkinter as tk
from math import cos, radians, sin

from mapdata import Map
from scandata import Scan
from scanLog import loadScanLog

DISTANCE_RATIO = 5  # 확대/축소
MOVE_RATIO = 50     # 이동
ROTATE_RATIO = 30   # 회전
RADIUS = 5          # 점 하나의 반지름
COLOR = ['blanched almond', 'blue violet', 'brown', 'burlywood', 'cadet blue', 'chartreuse', 'chocolate', 'coral', 'cornflower blue', 'dark goldenrod', 'dark green', 'dark olive green', 'dark orange', 'dark orchid', 'dark salmon']
 # 점 색상
CHECKED = []
CENTER_X = 0
CENTER_Y = 0

# 종료
def on_closing():
    root.destroy()

# 창 크기 조절시
def on_resize(event):
    global CENTER_X, CENTER_Y
    new_width = event.width
    new_height = event.height
    canvas.config(width=new_width, height=new_height)
    CENTER_X = new_width/2
    CENTER_Y = new_width/2
    draw_lidar_data()

# 마우스 좌클릭/우클릭으로 확대/축소 조절
def increase_distance_ratio(event):
    global DISTANCE_RATIO
    DISTANCE_RATIO += 3
    draw_lidar_data()
def decrease_distance_ratio(event):
    global DISTANCE_RATIO
    DISTANCE_RATIO -= 3
    if DISTANCE_RATIO < 1:
        DISTANCE_RATIO = 1
    draw_lidar_data()

# 방향키로 중심점 조절
def on_arrow_key(event):
    global CENTER_Y, CENTER_X
    key = event.keysym
    if key == 'Up':
        CENTER_Y += MOVE_RATIO
    elif key == 'Down':
        CENTER_Y -= MOVE_RATIO
    elif key == 'Left':
        CENTER_X += MOVE_RATIO
    elif key == 'Right':
        CENTER_X -= MOVE_RATIO
    draw_lidar_data()

# q, e 키입력으로 방향 조절
def rotate_left(event):
    rotate += ROTATE_RATIO
    draw_lidar_data()
def rotate_right(event):
    rotate -= ROTATE_RATIO
    draw_lidar_data()

# 좌표의 회전 계산
def rotate_cord(x,y):
    # x' = xcos - ysin, y' = xsin + ycos
    x = x - CENTER_X
    y = y - CENTER_Y
    sin0 = sin(radians(rotate))
    cos0 = cos(radians(rotate))
    x2 = x*cos0 - y*sin0 + CENTER_X
    y2 = x*sin0 + y*cos0 + CENTER_Y
    
    return x2, y2

# 체크 데이터 변경 후 그리기
def modify_check(index):
    if CHECKED[index].get() == 1:
        CHECKED[index].set(0)
        print(f'{index}번 선택 해제')
    else:
        CHECKED[index].set(1)
        print(f'{index}번 선택, color:{COLOR[index-1]}')
    draw_lidar_data()

# 스캔 데이터를 지도에 표시
def draw_lidar_data():
    canvas.delete("all")
    for index, value in enumerate(CHECKED):
        v = value.get()
        if v == 1:
            mapData = Map()
            mapData.update(Scan(logData[index], index))
            color = COLOR[index-1]

            '''
            for index, lineInfo in enumerate(mapData.lineInfo):
                x1 = self.CENTER_X + lineInfo.startX / DISTANCE_RATIO
                y1 = self.CENTER_Y + lineInfo.startY / DISTANCE_RATIO
                x2 = self.CENTER_X + lineInfo.endX / DISTANCE_RATIO
                y2 = self.CENTER_Y + lineInfo.endY  / DISTANCE_RATIO

                self.canvas.create_line(self.rotate_cord(x1, y1), self.rotate_cord(x2, y2), width=2, fill='red')
                # debug - 선마다 번호 표시
                midpoint = ((self.rotate_cord(x1, y1)[0] + self.rotate_cord(x2, y2)[0]) / 2, (self.rotate_cord(x1, y1)[1] + self.rotate_cord(x2, y2)[1]) / 2)
                self.canvas.create_text(midpoint, text=str(index))
            '''
            # 점의 형태로 지도 생성
            for index, cord in enumerate(mapData.cordInfo):
                try:
                    y1 = CENTER_Y + (cord.y - RADIUS) / DISTANCE_RATIO
                    x2 = CENTER_X + (cord.x + RADIUS) / DISTANCE_RATIO
                    y2 = CENTER_Y + (cord.y + RADIUS) / DISTANCE_RATIO
                    x1 = CENTER_X + (cord.x - RADIUS) / DISTANCE_RATIO

                    canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=color)
                except Exception as e:
                    x1 = CENTER_X + (cord[0] - RADIUS) / DISTANCE_RATIO
                    y1 = CENTER_Y + (cord[1] - RADIUS) / DISTANCE_RATIO
                    x2 = CENTER_X + (cord[0] + RADIUS) / DISTANCE_RATIO
                    y2 = CENTER_Y + (cord[1] + RADIUS) / DISTANCE_RATIO

                    canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=color)
                
            #     # 중심에 번호 표시
            #     CENTER_X_text = (x1 + x2) / 2
            #     CENTER_Y_text = (y1 + y2) / 2
            #     self.canvas.create_text(CENTER_X_text, CENTER_Y_text, text=str(index))

            # 중심점
            canvas.create_line(CENTER_X - RADIUS, CENTER_Y - RADIUS, CENTER_X + RADIUS, CENTER_Y + RADIUS, fill="red", arrow=tk.LAST)

if __name__ == '__main__':
    global root, canvas, logData, logName
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.title("DEBUG MAP")
    width = 1000
    height = 800
    canvas = tk.Canvas(root, width=width, height=height, bg="white")
    select = tk.Frame(root, width=width, bg='white')

    # 중심점 계산
    CENTER_X = width / 2
    CENTER_Y = height / 2
    # 창 크기 조절
    # root.bind("<Configure>", on_resize)
    # 확대/축소
    canvas.bind("<Button-1>", increase_distance_ratio)
    canvas.bind("<Button-3>", decrease_distance_ratio)
    # 이동
    root.bind("<Up>", on_arrow_key)
    root.bind("<Down>", on_arrow_key)
    root.bind("<Left>", on_arrow_key)
    root.bind("<Right>", on_arrow_key)
    # 회전
    rotate = 0
    root.bind("q", rotate_left)
    root.bind("e", rotate_right)

    select.pack()
    canvas.pack()

    ##### 로그
    logName, logData = loadScanLog()
    print(logName)
    for index, log in enumerate(logData):
        match = logName[index].split('-')
        button = tk.Checkbutton(select, text=match[-1].split('.')[0], bg='white', variable=index, command=lambda index=index: modify_check(index))
        CHECKED.append(tk.IntVar(0))
        button.grid(row=0, column=index, padx=10)
    
    root.mainloop()