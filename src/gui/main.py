# 메인 파일
import gui
import scanFile
import os

current_dir = os.getcwd()
path = os.path.join(current_dir, 'log', '2024-05-05')

gui.graphics_odometry([[0,0]])
data = scanFile.load(path)

gui.graphics_cord(data, 0.1)

