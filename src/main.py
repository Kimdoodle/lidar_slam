# 메인 파일
import gui
import scanFile

gui.graphics_odometry([[0,0]])
data = scanFile.load('../log')
for info in data:
    gui.graphics_cord(info, 'black')

gui.confirm()