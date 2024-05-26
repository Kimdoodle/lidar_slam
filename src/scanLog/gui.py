import os
import time
import tkinter as tk
from tkinter import filedialog

import pandas as pd


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
            self.display()

    def display(self):
        for file in self.csv_files:
            df = pd.read_csv(file)
            self.canvas.delete("all")

            for _, row in df.iterrows():
                x = row.iloc[0] * 30  # x 좌표 (미터 단위를 30배율로 조정)
                y = row.iloc[1] * 30  # y 좌표 (미터 단위를 30배율로 조정)
                self.canvas.create_oval(x + 400, y + 300, x + 402, y + 302, fill="blue")

            print(f"Displayed {file}")
            time.sleep(1)


if __name__ == '__main__':
    root = tk.Tk()
    app = CSVViewer(root)
    root.mainloop()
