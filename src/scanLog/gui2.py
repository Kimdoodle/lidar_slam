import os
import time
import tkinter as tk
from tkinter import filedialog

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class CSVViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("CSV Viewer")

        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

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
            self.display_next_file()

    def display_next_file(self):
        if self.current_index < len(self.csv_files):
            file = self.csv_files[self.current_index]
            df = pd.read_csv(file)
            
            x = df.iloc[:, 0] * 30
            y = df.iloc[:, 1] * 30
            
            self.ax.clear()
            self.ax.scatter(x, y, c='blue')
            self.ax.set_title(f"Displaying {file}")
            self.canvas.draw()

            self.current_index += 1
            self.root.after(500, self.display_next_file)  # update every 0.5 seconds


if __name__ == '__main__':
    root = tk.Tk()
    app = CSVViewer(root)
    root.mainloop()
