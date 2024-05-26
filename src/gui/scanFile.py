import os
import pandas as pd

def load(path: str):
    try:
        fileList = os.listdir(path)
        if not fileList:
            raise FileNotFoundError(f"No files found in directory: {path}")

        fileData = []
        for file in fileList:
            df = pd.read_csv(os.path.join(path, file), usecols=[0, 1])
            data_list = [tuple(map(float, x)) for x in df.to_records(index=False)]
            fileData.append(data_list)

        for i in range(len(fileData)):
            yield fileData[i]

    except Exception as e:
        print(e)
        return None
