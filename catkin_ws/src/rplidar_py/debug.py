# 로그 추출 및 디버깅용 파일
import re
import tkinter as tk
from math import cos, radians, sin

from mapdata import Map
from scandata import Scan
from scanLog import loadScanLog

DISTANCE_RATIO = 5  # 확대/축소
MOVE_RATIO = 10  # 이동
ROTATE_RATIO = 1  # 회전
RADIUS = 5  # 점 하나의 반지름
MODE = 'line' # 출력 모드 : dot/line
COLOR = [
    "blanched almond",
    "blue violet",
    "brown",
    "burlywood",
    "cadet blue",
    "chartreuse",
    "chocolate",
    "coral",
    "cornflower blue",
    "dark goldenrod",
    "dark green",
    "dark olive green",
    "dark orange",
    "dark orchid",
    "dark salmon",
]
# 점 색상
CHECK = 0
CHECKED = []
CENTER_X = 0
CENTER_Y = 0
MOVE_X = 0
MOVE_Y = 0
ROTATE = 0


# 종료
def on_closing():
    root.destroy()

# 창 크기 조절시
def on_resize(event):
    global CENTER_X, CENTER_Y, mapData
    new_width = event.width
    new_height = event.height
    canvas.config(width=new_width, height=new_height)
    CENTER_X = new_width / 2
    CENTER_Y = new_width / 2
    draw_lidar_data(mapData)

# 마우스 좌클릭/우클릭으로 확대/축소 조절
def increase_distance_ratio(event):
    global DISTANCE_RATIO, mapData
    DISTANCE_RATIO += 3
    draw_lidar_data(mapData)
def decrease_distance_ratio(event):
    global DISTANCE_RATIO, mapData
    DISTANCE_RATIO -= 3
    if DISTANCE_RATIO < 1:
        DISTANCE_RATIO = 1
    draw_lidar_data(mapData)

# 방향키로 중심점 조절
def on_arrow_key(event):
    global MOVE_X, MOVE_Y, mapData
    key = event.keysym
    if key == "Up":
        MOVE_Y += MOVE_RATIO
    elif key == "Down":
        MOVE_Y -= MOVE_RATIO
    elif key == "Left":
        MOVE_X += MOVE_RATIO
    elif key == "Right":
        MOVE_X -= MOVE_RATIO
    draw_lidar_data(mapData)

# q, e 키입력으로 방향 조절
def rotate_left(event):
    global ROTATE, mapData
    ROTATE += ROTATE_RATIO
    draw_lidar_data(mapData)
def rotate_right(event):
    global ROTATE, mapData
    ROTATE -= ROTATE_RATIO
    draw_lidar_data(mapData)

# 좌표의 회전 계산
def rotate_cord(x, y):
    global CENTER_X, CENTER_Y, ROTATE
    # x' = xcos - ysin, y' = xsin + ycos
    x = x - CENTER_X
    y = y - CENTER_Y
    sin0 = sin(radians(ROTATE))
    cos0 = cos(radians(ROTATE))
    x2 = x * cos0 - y * sin0 + CENTER_X
    y2 = x * sin0 + y * cos0 + CENTER_Y

    return x2, y2

# 체크 데이터 변경 후 그리기
def modify_check(index):
    global CHECK, mapData
    if CHECKED[index].get() == 1:
        CHECKED[index].set(0)
        print(f"{index}번 선택 해제")
    else:
        CHECKED[index].set(1)
        CHECK = index
        print(f"{index}번 선택, data length: {len(logData[index])}")
    draw_lidar_data(mapData)

# 모드 변경
def modeChange():
    global MODE, mapData
    if MODE == 'dot':
        MODE = 'line'
    elif MODE == 'line':
        MODE = 'dot'
    draw_lidar_data(mapData)

# 스캔 데이터를 지도에 표시
def draw_0():
    global MODE
    if MODE == 'dot':
        mapData.update(Scan(logData[0]))
        # 점의 형태로 지도 생성
        for index, cord in enumerate(mapData.cordInfo):
            x1 = CENTER_X + (cord.x - RADIUS) / DISTANCE_RATIO
            y1 = CENTER_Y + (cord.y - RADIUS) / DISTANCE_RATIO
            x2 = CENTER_X + (cord.x + RADIUS) / DISTANCE_RATIO
            y2 = CENTER_Y + (cord.y + RADIUS) / DISTANCE_RATIO
            canvas.create_rectangle(x1, y1, x2, y2, fill='black')
    elif MODE == 'line':
        # 선의 형태로 지도 생성
        for index, lineInfo in enumerate(mapData.lineInfo):
            x1 = CENTER_X + lineInfo.startX / DISTANCE_RATIO + MOVE_X
            y1 = CENTER_Y + lineInfo.startY / DISTANCE_RATIO + MOVE_Y
            x2 = CENTER_X + lineInfo.endX / DISTANCE_RATIO + MOVE_X
            y2 = CENTER_Y + lineInfo.endY / DISTANCE_RATIO + MOVE_Y
            newCord1 = rotate_cord(x1, y1)
            newCord2 = rotate_cord(x2, y2)

            canvas.create_line(newCord1, newCord2, width=2, fill="black")


def draw_lidar_data(mapData:Map):
    global MODE
    canvas.delete("all")
    print(f"MOVE_X: {MOVE_X}, MOVE_Y: {MOVE_Y}, ROTATE: {ROTATE}")
    removeRadius = RADIUS * 100 / DISTANCE_RATIO
    for index, value in enumerate(CHECKED):
        # 0번 데이터는 항상 출력
        if index == 0:
            draw_0()
        else:
            # 나머지 데이터
            v = value.get()
            if v == 1:
                mapData.update(Scan(logData[index]))
                color = COLOR[index]

                if MODE == 'dot':
                    # 점의 형태로 지도 생성
                    for index, cord in enumerate(mapData.cordInfo):
                        y1 = CENTER_Y + (cord.y - RADIUS) / DISTANCE_RATIO + MOVE_Y
                        x2 = CENTER_X + (cord.x + RADIUS) / DISTANCE_RATIO + MOVE_X
                        y2 = CENTER_Y + (cord.y + RADIUS) / DISTANCE_RATIO + MOVE_Y
                        x1 = CENTER_X + (cord.x - RADIUS) / DISTANCE_RATIO + MOVE_X

                        canvas.create_rectangle(rotate_cord(x1,y1), rotate_cord(x2,y2), fill=color, outline=color)

                        # # 중심에 번호 표시
                        # CENTER_X_text = (x1 + x2) / 2
                        # CENTER_Y_text = (y1 + y2) / 2
                        # canvas.create_text(CENTER_X_text, CENTER_Y_text, text=str(index))
                elif MODE == 'line':
                    # 선의 형태로 지도 생성
                    for index, lineInfo in enumerate(mapData.lineInfo):
                        x1 = CENTER_X + lineInfo.startX / DISTANCE_RATIO + MOVE_X
                        y1 = CENTER_Y + lineInfo.startY / DISTANCE_RATIO + MOVE_Y
                        x2 = CENTER_X + lineInfo.endX / DISTANCE_RATIO + MOVE_X
                        y2 = CENTER_Y + lineInfo.endY / DISTANCE_RATIO + MOVE_Y
                        newCord1 = rotate_cord(x1, y1)
                        newCord2 = rotate_cord(x2, y2)

                        canvas.create_line(newCord1, newCord2, width=2, fill=color)
                        # debug - 선마다 번호 표시
                        midpoint = (
                            (newCord1[0] + newCord2[0]) / 2,
                            (newCord1[1] + newCord2[1]) / 2,
                        )
                        #canvas.create_text(midpoint, text=str(index))



    # 중심점
    canvas.create_line(
        CENTER_X - 2 * RADIUS,
        CENTER_Y,
        CENTER_X + 2 * RADIUS,
        CENTER_Y,
        fill="red",
        arrow=tk.LAST,
    )

    canvas.create_oval(
        CENTER_X - removeRadius,
        CENTER_Y - removeRadius,
        CENTER_X + removeRadius,
        CENTER_Y + removeRadius,
        fill=None,
    )


if __name__ == "__main__":
    global root, canvas, logData, logName, mapData
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.title("DEBUG MAP")
    width = 1000
    height = 800
    canvas = tk.Canvas(root, width=width, height=height, bg="white")
    select = tk.Frame(root, width=width)
    modeFrame = tk.Frame(root, width=width)

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
    root.bind("q", rotate_left)
    root.bind("e", rotate_right)

    select.pack()
    modeFrame.pack()
    canvas.pack()

    # 로그
    logName, logData = loadScanLog()
    # 지도 데이터 선언
    mapData = Map(Scan(logData[0]))

    # 표시하는 로그 데이터 버튼
    for index, log in enumerate(logData):
        intvalue = 0
        if index == 0:
            # 0번 데이터 항상 활성화
            intvalue = 1

        match = logName[index].split("-")
        button = tk.Checkbutton(
            select,
            text=match[-1].split(".")[0],
            variable=index,
            command=lambda index=index: modify_check(index),
        )
        CHECKED.append(tk.IntVar(value=intvalue))
        # print(f"index:{index}, intvalue:{intvalue}")
        button.grid(row=0, column=index, padx=10)

    # 모드 라디오버튼
    radio_var = tk.StringVar()
    modeButton1 = tk.Radiobutton(modeFrame, text='점으로 표시', variable=radio_var, value='dot', command=modeChange)
    modeButton1.grid(column=0, row=1)
    modeButton2 = tk.Radiobutton(modeFrame, text='선으로 표시', variable=radio_var, value='line', command=modeChange)
    modeButton2.grid(column=1, row=1)

    draw_lidar_data(mapData)
    root.mainloop()
