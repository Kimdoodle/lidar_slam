import tkinter as tk
from tkinter import filedialog
import pandas as pd
import threading
import time
import os

class CSVViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("CSV Viewer")

        self.canvas = tk.Canvas(self.root, width=800, height=600, bg="white")
        self.canvas.pack()

        self.select_button = tk.Button(self.root, text="Select Folder", command=self.select_folder)
        self.select_button.pack()

        self.csv_files = []
        self.current_index = 0
        self.displaying = False

    def select_folder(self):
        folder_path = filedialog.askdirectory()
        if folder_path:
            self.csv_files = sorted([os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith('.csv')])
            self.current_index = 0
            self.start_displaying()

    def start_displaying(self):
        if not self.displaying:
            self.displaying = True
            threading.Thread(target=self.display_csv_files).start()

    def display_csv_files(self):
        while self.displaying and self.current_index < len(self.csv_files):
            self.display_csv(self.csv_files[self.current_index])
            self.current_index += 1
            time.sleep(3)
        self.displaying = False

    def display_csv(self, file_path):
        df = pd.read_csv(file_path)
        self.canvas.delete("all")

        for _, row in df.iterrows():
            x = row[0] * 30  # x 좌표 (미터 단위를 100배율로 조정)
            y = row[1] * 30  # y 좌표 (미터 단위를 100배율로 조정)
            self.canvas.create_oval(x + 400, y + 300, x + 402, y + 302, fill="blue")
            
        print(f"Displayed {file_path}")
        
if __name__ == '__main__':
    root = tk.Tk()
    app = CSVViewer(root)
    root.mainloop()
