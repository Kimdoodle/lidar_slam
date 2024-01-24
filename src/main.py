# 로그 추출 및 디버깅용 파일
import re
import tkinter as tk
from math import cos, radians, sin

from calculate import midCord
from mapdata import Map
from scandata import Scan
from scanLog import loadScanLog

DISTANCE_RATIO = 3  # 확대/축소
MOVE_RATIO = 30  # 이동
ROTATE_RATIO = 5  # 회전
RADIUS = 5  # 점 하나의 반지름
MODE = 'dot' # 출력 모드 : dot/line/debug
COLOR = [ "black", "blue violet", "brown", "burlywood", "cadet blue",
    "chartreuse", "chocolate", "coral", "cornflower blue", "dark goldenrod",
    "dark green", "dark olive green", "dark orange", "dark orchid", "dark salmon",
] # 점 색상
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
    global CENTER_X, CENTER_Y
    new_width = event.width
    new_height = event.height
    canvas.config(width=new_width, height=new_height)
    CENTER_X = new_width / 2
    CENTER_Y = new_width / 2
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
    global MOVE_X, MOVE_Y
    key = event.keysym
    if key == "Up":
        MOVE_Y += MOVE_RATIO
    elif key == "Down":
        MOVE_Y -= MOVE_RATIO
    elif key == "Left":
        MOVE_X += MOVE_RATIO
    elif key == "Right":
        MOVE_X -= MOVE_RATIO
    draw_lidar_data()

# q, e 키입력으로 방향 조절
def rotate_left(event):
    global ROTATE
    ROTATE += ROTATE_RATIO
    draw_lidar_data()
def rotate_right(event):
    global ROTATE
    ROTATE -= ROTATE_RATIO
    draw_lidar_data()

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
    global CHECK
    if CHECKED[index].get() == 1:
        CHECKED[index].set(0)
        print(f"{index}번 선택 해제")
    else:
        CHECKED[index].set(1)
        CHECK = index
        print(f"{index}번 선택")
    draw_lidar_data()

# 모드 변경
def modeChange():
    global MODE
    MODE = radio_var.get()
    draw_lidar_data()

# 번호 표시 변경
def numberChange():
    global printNumber
    if printNumber.get() == 0:
        printNumber.set(1)
    else:
        printNumber.set(0)
    draw_lidar_data()

# 스캔 데이터를 지도에 표시
'''
    MODE = dot --> 선택한 데이터 각각을 점 데이터로 표시함
    MODE = line --> 선택한 데이터를 병합하여 선 데이터로 표시함
    MODE = debug --> 선택한 데이터의 선 데이터 정보를 각각 표시함
'''
def draw_lidar_data():
    canvas.delete("all")
    print(f"MOVE_X: {MOVE_X}, MOVE_Y: {MOVE_Y}, ROTATE: {ROTATE}")

    logData = loadScanLog()[1]

    if MODE == 'dot':
        # 점의 형태로 지도 생성
        for index, log in enumerate(logData):
            if CHECKED[index].get() == 0: continue
            color = COLOR[index]
            mapData = Map(Scan(log))
            for index, cord in enumerate(mapData.cordInfo):
                y1 = CENTER_Y + (cord.y - RADIUS) / DISTANCE_RATIO + MOVE_Y
                x2 = CENTER_X + (cord.x + RADIUS) / DISTANCE_RATIO + MOVE_X
                y2 = CENTER_Y + (cord.y + RADIUS) / DISTANCE_RATIO + MOVE_Y
                x1 = CENTER_X + (cord.x - RADIUS) / DISTANCE_RATIO + MOVE_X

                canvas.create_rectangle(rotate_cord(x1,y1), rotate_cord(x2,y2), fill=color, outline=color)

                # 중심에 번호 표시
                if printNumber.get() == 1:
                    CENTER_X_text = (x1 + x2) / 2 + 3*RADIUS
                    CENTER_Y_text = (y1 + y2) / 2
                    canvas.create_text(CENTER_X_text, CENTER_Y_text, text=str(index))

    elif MODE == 'line':
        # 체크된 데이터 병합
        mapData = Map(Scan(logData[0]))
        for index, value in enumerate(CHECKED[1:]):
            if value.get() == 1:
                mapData.update(Scan(logData[index+1]))
        # 선의 형태로 지도 생성
        color = 'red'
        for index, lineInfo in enumerate(mapData.lineInfo):
            x1 = CENTER_X + lineInfo.startX / DISTANCE_RATIO + MOVE_X
            y1 = CENTER_Y + lineInfo.startY / DISTANCE_RATIO + MOVE_Y
            x2 = CENTER_X + lineInfo.endX / DISTANCE_RATIO + MOVE_X
            y2 = CENTER_Y + lineInfo.endY / DISTANCE_RATIO + MOVE_Y
            newCord1 = rotate_cord(x1, y1)
            newCord2 = rotate_cord(x2, y2)

            canvas.create_line(newCord1, newCord2, width=2, fill=color)

            # 선마다 번호 표시
            if printNumber.get() == 1:
                midpoint = midCord(newCord1, newCord2)
                canvas.create_text(midpoint, text=str(index))

    elif MODE == 'debug':
        # 선의 형태로 지도 생성
        for index, log in enumerate(logData):
            if CHECKED[index].get() == 0: continue
            color = 'red' if index==0 else COLOR[index]
            mapData = Map(Scan(log))
            for index, lineInfo in enumerate(mapData.lineInfo):
                x1 = CENTER_X + lineInfo.startX / DISTANCE_RATIO + MOVE_X
                y1 = CENTER_Y + lineInfo.startY / DISTANCE_RATIO + MOVE_Y
                x2 = CENTER_X + lineInfo.endX / DISTANCE_RATIO + MOVE_X
                y2 = CENTER_Y + lineInfo.endY / DISTANCE_RATIO + MOVE_Y
                newCord1 = rotate_cord(x1, y1)
                newCord2 = rotate_cord(x2, y2)

                canvas.create_line(newCord1, newCord2, width=2, fill=color)

                # 선마다 번호 표시
                if printNumber.get() == 1:
                    midpoint = midCord(newCord1, newCord2)
                    canvas.create_text(midpoint, text=str(index))


    # 중심점
    canvas.create_line(
        CENTER_X - 2 * RADIUS + MOVE_X,
        CENTER_Y + MOVE_Y,
        CENTER_X + 2 * RADIUS + MOVE_X,
        CENTER_Y + MOVE_Y,
        fill="red",
        arrow=tk.LAST,
    )
    # dist = 500 / DISTANCE_RATIO
    # canvas.create_oval(
    #     CENTER_X - dist, CENTER_Y - dist,
    #     CENTER_X + dist, CENTER_Y + dist,
    #     fill=None
    # )


if __name__ == "__main__":
    global root, canvas
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.title("DEBUG MAP")
    width = 1600
    height = 900
    # width = 3000
    # height = 2000
    canvas = tk.Canvas(root, width=width, height=height, bg="white")
    select = tk.Frame(root, width=width)
    modeFrame = tk.Frame(root, width=width)
    infoFrame = tk.Frame(root, width=width)

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
    infoFrame.pack()

    # 로그
    logName = loadScanLog()[0]

    # 로그 데이터 병합 선택 버튼
    for index, log in enumerate(logName):
        intvalue = 0

        match = logName[index].split("-")
        button = tk.Checkbutton(
            select,
            text=match[-1].split(".")[0],
            variable=index,
            command=lambda index=index: modify_check(index),
        )
        if index == 0:
            # 0번 데이터 항상 활성화
            button.configure(state='disabled')
            intvalue = 1

        CHECKED.append(tk.IntVar(value=intvalue))
        # print(f"index:{index}, intvalue:{intvalue}")
        button.grid(row=0, column=index, padx=10)

    # 모드 선택 버튼
    global radio_var
    radio_var = tk.StringVar(value=MODE)
    modeButton1 = tk.Radiobutton(modeFrame, text='점으로 표시', variable=radio_var, value='dot', command=modeChange)
    modeButton1.grid(column=0, row=1)
    modeButton2 = tk.Radiobutton(modeFrame, text='선 병합', variable=radio_var, value='line', command=modeChange)
    modeButton2.grid(column=1, row=1)
    modeButton3 = tk.Radiobutton(modeFrame, text='선 개별 표시', variable=radio_var, value='debug', command=modeChange)
    modeButton3.grid(column=2, row=1)

    global printNumber
    printNumber = tk.IntVar(value=0)
    numberButton = tk.Checkbutton(modeFrame, text='번호 표시', command=numberChange)
    numberButton.grid(column=3, row=1, columnspan=2)

    # 안내사항
    label = tk.Label(infoFrame, text='q/e: 회전, 좌/우클릭: 확대/축소, 화살표: 이동').grid()
    draw_lidar_data()
    root.mainloop()
