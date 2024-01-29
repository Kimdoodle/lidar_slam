# 로그 추출 및 디버깅용 파일
import re
import tkinter as tk
from math import cos, radians, sin

from calculate import midCord
from mapdata import Map
from scandata import Scan
from scanLog import loadScanLog

WIDTH = 1600
HEIGHT = 900
ROOT = tk.Tk()
CANVAS = tk.Canvas(ROOT, width=WIDTH, height=HEIGHT, bg="white")
DISTANCE_RATIO = 3  # 확대/축소
MOVE_RATIO = 30  # 이동
ROTATE_RATIO = 5  # 회전
RADIUS = 5  # 점 하나의 반지름

MODE = [tk.IntVar(value=0), tk.IntVar(value=0)]  # 모드, 상세 단계 번호
MODE_NAMES = ['기본', '실험1', '실험2']
MODES = [['점 표시'],
         ['다운샘플링', 'ICP', '결합'],
         ['선 생성', '선 조합', '결합']]
BUTTONS = [[], [], []]
PRINT = tk.IntVar(value=0)

COLOR = ["black", "blue violet", "brown", "burlywood", "cadet blue",
         "chartreuse", "chocolate", "coral", "cornflower blue", "dark goldenrod",
         "dark green", "dark olive green", "dark orange", "dark orchid", "dark salmon", ]  # 점 색상

CHECKED = []
CENTER_X = 0
CENTER_Y = 0
MOVE_X = 0
MOVE_Y = 0
ROTATE = 0


# 종료
def on_closing():
    ROOT.destroy()


# 창 크기 조절시
def on_resize(event):
    global CENTER_X, CENTER_Y
    new_width = event.WIDTH
    new_height = event.HEIGHT
    CANVAS.config(width=new_width, height=new_height)
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
def modify_check(value):
    global CHECKED
    if CHECKED[value].get() == 1:
        CHECKED[value].set(0)
        print(f"{value}번 선택 해제")
    else:
        CHECKED[value].set(1)
        print(f"{value}번 선택")
    # print([c.get() for c in CHECKED])
    draw_lidar_data()

# 모드 변경
def modeChange():
    curMode = MODE[0].get()
    for buttonIndex, buttons in enumerate(BUTTONS):
        if buttonIndex == curMode:
            for button in buttons:
                button.config(state=tk.NORMAL)
        else:
            for button in buttons:
                button.config(state=tk.DISABLED)
    print(f"현재 모드: {MODE_NAMES[MODE[0].get()]} {MODES[MODE[0].get()][MODE[1].get()]}")
    draw_lidar_data()


# 번호 표시 변경
def numberChange():
    global PRINT
    if PRINT.get() == 0:
        PRINT.set(1)
    else:
        PRINT.set(0)
    draw_lidar_data()


'''
    스캔 데이터를 지도에 표시
    MODE = 기본 --> 선택한 데이터를 각각 점 데이터로 표시함
    MODE = 실험1 --> 선택한 데이터를 점 기준으로 병합하는 과정
    MODE = 실험2 --> 선택한 데이터를 선 기준으로 병합하는 과정
'''
def draw_lidar_data():
    CANVAS.delete("all")
    # print(f"MOVE_X: {MOVE_X}, MOVE_Y: {MOVE_Y}, ROTATE: {ROTATE}")

    logData = loadScanLog()[1]
    mapdata = Map(Scan(logData[0]))  # 기본 지도 데이터 정의
    for index, checkData in enumerate(CHECKED):
        if index == 0: continue
        if checkData.get() == 1:
            mapdata.simpleAdd(logData[index])
            mapdata.update(logData[index])
    if MODE[0].get() == 0:  # 기본
        for i, scanLog in enumerate(mapdata.scanDataLog):
            printDot(scanLog.cordInfo, COLOR[i])

            # Todo: 1/28
    elif MODE[0].get() == 1:  # ICP
        if MODE[1].get() == 0:  # 다운샘플링 - 2개의 (x,y) list
            printDot(mapdata.icpStep1_DownSample[0])
            printDot(mapdata.icpStep1_DownSample[1])

        if MODE[1].get() >= 1:
            print(mapdata.icpStep2_ICPResult)

        if MODE[1].get() == 2:
            printDot(mapdata.icpStep3_MergedData)

    elif MODE[0].get() == 2: # MY
        if MODE[1].get() == 0: # 기본 선 데이터
            printLine(mapdata.myStep1_makeLine[0], 'black')


# 점으로 그리기
def printDot(logData: list, color='black'):
    for index, cord in enumerate(logData):
        try:
            x1 = CENTER_X + (cord.x - RADIUS) / DISTANCE_RATIO + MOVE_X
            y1 = CENTER_Y + (cord.y - RADIUS) / DISTANCE_RATIO + MOVE_Y
            x2 = CENTER_X + (cord.x + RADIUS) / DISTANCE_RATIO + MOVE_X
            y2 = CENTER_Y + (cord.y + RADIUS) / DISTANCE_RATIO + MOVE_Y

            CANVAS.create_rectangle(rotate_cord(x1, y1), rotate_cord(x2, y2),
                                    fill=color, outline=color)
        except Exception as e:
            x1 = CENTER_X + (cord.x - RADIUS) / DISTANCE_RATIO + MOVE_X
            y1 = CENTER_Y + (cord.y - RADIUS) / DISTANCE_RATIO + MOVE_Y
            x2 = CENTER_X + (cord.x + RADIUS) / DISTANCE_RATIO + MOVE_X
            y2 = CENTER_Y + (cord.y + RADIUS) / DISTANCE_RATIO + MOVE_Y

            CANVAS.create_rectangle(rotate_cord(x1, y1), rotate_cord(x2, y2),
                                    fill=color, outline=color)

# 선으로 그리기
def printLine(lineData:list, color='black'):
        for index, lineInfo in enumerate(lineData):
            x1 = CENTER_X + lineInfo.startX / DISTANCE_RATIO + MOVE_X
            y1 = CENTER_Y + lineInfo.startY / DISTANCE_RATIO + MOVE_Y
            x2 = CENTER_X + lineInfo.endX / DISTANCE_RATIO + MOVE_X
            y2 = CENTER_Y + lineInfo.endY / DISTANCE_RATIO + MOVE_Y
            newCord1 = rotate_cord(x1, y1)
            newCord2 = rotate_cord(x2, y2)

            CANVAS.create_line(newCord1, newCord2, width=2, fill=color)

            # 선마다 번호 표시
            if PRINT.get() == 1:
                midpoint = midCord(newCord1, newCord2)
                CANVAS.create_text(midpoint, text=str(index))

        # 중심 화살표
        CANVAS.create_line(
            CENTER_X - 2 * RADIUS + MOVE_X, CENTER_Y + MOVE_Y,
            CENTER_X + 2 * RADIUS + MOVE_X, CENTER_Y + MOVE_Y,
            fill="red", arrow=tk.LAST,
        )

    # if MODE == 'dot':
    #     # 점의 형태로 지도 생성
    #
    # 
    # elif MODE == 'line':
    #     # 체크된 데이터 병합
    #     mapData = Map(Scan(logData[0]))
    #     for index, value in enumerate(CHECKED[1:]):
    #         if value.get() == 1:
    #             mapData.update(Scan(logData[index + 1]))
    #
    # 
    # elif MODE == 'debug':
    #     # 선의 형태로 지도 생성
    #     for index, log in enumerate(logData):
    #         if CHECKED[index].get() == 0: continue
    #         color = 'red' if index == 0 else COLOR[index]
    #         mapData = Map(Scan(log))
    #         for index, lineInfo in enumerate(mapData.lineInfo):
    #             x1 = CENTER_X + lineInfo.startX / DISTANCE_RATIO + MOVE_X
    #             y1 = CENTER_Y + lineInfo.startY / DISTANCE_RATIO + MOVE_Y
    #             x2 = CENTER_X + lineInfo.endX / DISTANCE_RATIO + MOVE_X
    #             y2 = CENTER_Y + lineInfo.endY / DISTANCE_RATIO + MOVE_Y
    #             newCord1 = rotate_cord(x1, y1)
    #             newCord2 = rotate_cord(x2, y2)
    # 
    #             CANVAS.create_line(newCord1, newCord2, width=2, fill=color)
    # 
    #             # 선마다 번호 표시
    #             if PRINT.get() == 1:
    #                 midpoint = midCord(newCord1, newCord2)
    #                 CANVAS.create_text(midpoint, text=str(index))



    # dist = 500 / DISTANCE_RATIO
    # canvas.create_oval(
    #     CENTER_X - dist, CENTER_Y - dist,
    #     CENTER_X + dist, CENTER_Y + dist,
    #     fill=None
    # )


if __name__ == "__main__":
    ROOT.protocol("WM_DELETE_WINDOW", on_closing)
    ROOT.title("DEBUG MAP")

    select = tk.Frame(ROOT, width=WIDTH)
    modeFrame = tk.Frame(ROOT, width=WIDTH)
    infoFrame = tk.Frame(ROOT, width=WIDTH)

    # 중심점 계산
    CENTER_X = WIDTH / 2
    CENTER_Y = HEIGHT / 2
    # 창 크기 조절
    # root.bind("<Configure>", on_resize)
    # 확대/축소
    CANVAS.bind("<Button-1>", increase_distance_ratio)
    CANVAS.bind("<Button-3>", decrease_distance_ratio)
    # 이동
    ROOT.bind("<Up>", on_arrow_key)
    ROOT.bind("<Down>", on_arrow_key)
    ROOT.bind("<Left>", on_arrow_key)
    ROOT.bind("<Right>", on_arrow_key)
    # 회전
    ROOT.bind("q", rotate_left)
    ROOT.bind("e", rotate_right)

    select.pack()
    modeFrame.pack()
    CANVAS.pack()
    infoFrame.pack()

    # 로그
    logName = loadScanLog()[0]
    # 로그 데이터 선택 버튼
    for logIndex, log in enumerate(logName):
        intvalue = tk.IntVar(value=0)
        CHECKED.append(intvalue)
        match = logName[logIndex].split("-")
        button = tk.Checkbutton(select, text=match[-1].split(".")[0],
                                command=lambda i=logIndex: modify_check(i))
        if logIndex == 0:
            # 0번 데이터 항상 활성화
            button.configure(state='disabled', variable=intvalue)
            intvalue.set(1)

        # print(f"index:{logIndex}, intvalue:{intvalue}")
        button.grid(row=0, column=logIndex, padx=10)

    # 모드 선택 버튼
    numberButton = tk.Checkbutton(modeFrame, text='번호 표시', command=numberChange)
    numberButton.grid(column=0, row=0, rowspan=3)

    for index, modeName in enumerate(MODE_NAMES):
        modeButton = tk.Radiobutton(modeFrame, text=modeName, variable=MODE[0],
                                    value=index, command=modeChange)
        modeButton.grid(row=index, column=1)
        for stepIndex, mode in enumerate(MODES[index]):
            stepButton = tk.Radiobutton(modeFrame, text=mode, variable=MODE[1],
                                        value=stepIndex, command=modeChange)
            stepButton.grid(row=index, column=stepIndex + 2)
            BUTTONS[index].append(stepButton)
    modeChange()

    # 안내사항
    label = (tk.Label(infoFrame, text='q/e: 회전, 좌/우클릭: 확대/축소, 화살표: 이동'))
    label.grid()
    draw_lidar_data()
    ROOT.mainloop()
