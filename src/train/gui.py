# 데이터 클래스 결정용 GUI
import tkinter as tk
import sys

import numpy as np

sys.path.append('../')
from gui import scanFile
import calculate as cal


WIDTH = 1600
HEIGHT = 900
CENTERX = WIDTH / 2
CENTERY = HEIGHT / 2
RADIUS = 2
RATIO = 0.2
ROTATE = 10

INDEX = 0       # 데이터 전체를 선택하는 인덱스
SUBCORD = []    # 한 데이터의 좌표 데이터
SUBLINE = []    # 한 데이터의 선 데이터
SUBTRAIN = []   # 한 데이터의 훈련 데이터
SUBLENGTH = 0   # 한 데이터의 크기
SUBINDEX = 0    # 한 데이터 내 선을 선택하는 인덱스

ROOT = tk.Tk()
CANVAS = tk.Canvas(ROOT, width=WIDTH, height=HEIGHT, bg="white")
COLORS = ['black', 'red']


# 종료 함수. 종료 시 데이터를 csv파일에 저장함
def on_closing():
    # Todo: csv파일로 저장
    ROOT.destroy()


def onSave(data: list):
    global INDEX, SUBCORD, SUBTRAIN, SUBLENGTH, SUBINDEX
    try:
        INDEX = INDEX + 1
        newData = dataList[INDEX]
        SUBCORD = []
        SUBTRAIN = []
        SUBLENGTH = len(newData)
        SUBINDEX = 0

        CANVAS.delete("all")
        makeCords(newData)
        draw()
    except IndexError as e:
        # Todo: 데이터를 저장할 csv파일에 추가
        print("All log saved.")
        on_closing()


# 결정
def selLine(event):
    global SUBINDEX, SUBCORD
    # 회전 처리
    if event.keysym == 'q':
        for i in range(len(SUBCORD)):
            SUBCORD[i] = cal.rotateCord(SUBCORD[i], ROTATE)
        SUBINDEX = 0
        draw()
        return
    elif event.keysym == 'e':
        for i in range(len(SUBCORD)):
            SUBCORD[i] = cal.rotateCord(SUBCORD[i], -ROTATE)
        SUBINDEX = 0
        draw()
        return

    # 입력 처리
    x0, y0 = SUBCORD[SUBINDEX]
    x1, y1 = SUBCORD[(SUBINDEX + 1) % SUBLENGTH]
    x2, y2 = SUBCORD[(SUBINDEX + 2) % SUBLENGTH]

    dot1 = dataList[INDEX][SUBINDEX]
    dot2 = dataList[INDEX][(SUBINDEX + 1) % SUBLENGTH]

    angleDiff = np.abs(dot1[-2] - dot2[-2])
    distanceFromCenterDiff = np.abs(dot1[-1] - dot2[-1])
    distanceDiff = np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

    res = 0 if event.keysym == 'space' else 1
    if res == 1:
        line = SUBLINE[-1]
        CANVAS.itemconfig(line, fill='red')

    line = CANVAS.create_line(x1 * RATIO + CENTERX, y1 * RATIO + CENTERY,
                              x2 * RATIO + CENTERX, y2 * RATIO + CENTERY, fill='blue', width=3)

    SUBTRAIN.insert(SUBINDEX, (angleDiff, distanceDiff, distanceFromCenterDiff, res))
    SUBLINE.append(line)
    SUBINDEX = (SUBINDEX + 1) % SUBLENGTH



# 좌표 계산
def makeCords(data: list):
    global SUBLENGTH, SUBCORD
    SUBLENGTH = len(data)
    for i in range(SUBLENGTH):
        dot = data[i]
        dotCord = scanFile.calXY(dot[-2], dot[-1])
        SUBCORD.append(dotCord)

# 좌표 그리기
def draw():
    CANVAS.delete('all')
    CANVAS.create_oval(CENTERX - 5, CENTERY - 5, CENTERX + 5, CENTERY + 5, fill='red')
    for cord in SUBCORD:
        c1, c2 = cal.ovalCord(cal.moveCord(cord, RATIO), RADIUS)
        x1, y1 = c1[0] + CENTERX, c1[1] + CENTERY
        x2, y2 = c2[0] + CENTERX, c2[1] + CENTERY
        CANVAS.create_oval(x1, y1, x2, y2, fill='black')

    # 첫번째 선
    x1, y1 = SUBCORD[0]
    x2, y2 = SUBCORD[1]
    line = CANVAS.create_line(x1 * RATIO + CENTERX, y1 * RATIO + CENTERY,
                              x2 * RATIO + CENTERX, y2 * RATIO + CENTERY, fill='blue', width=5)
    SUBLINE.append(line)


# 1. 데이터를 로딩(txt파일)
dataList = scanFile.load("../../log2")

# 2. GUI를 생성
ROOT.title("Data Train Map")
ROOT.protocol("WM_DELETE_WINDOW", on_closing)
'''
    frame은 총 2개
    1. 파일 선택 ,현재 데이터 저장 프레임
    2. 좌표 표시, 체크 프레임
'''
selectFrame = tk.Frame(ROOT, width=WIDTH)
selectFrame.pack()
saveButton = tk.Button(selectFrame, text="완료",
                       command=lambda i=dataList[INDEX]: onSave(i))
saveButton.pack()

CANVAS.pack()
makeCords(dataList[INDEX])
draw()
ROOT.bind("<KeyPress>", selLine)

ROOT.mainloop()
