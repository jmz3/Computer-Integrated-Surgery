from ast import main
import numpy as np
import pandas as pd

def LoadTxtData(file_name):
    data = np.loadtxt(file_name,skiprows=1,delimiter=',')
    data_info = np.loadtxt(file_name,max_rows=1,dtype=str,delimiter=',')
    return data,data_info



# if __name__ == "__main__":
#     data = LoadTxtData('PA1 Student Data/pa1-debug-a-calbody.txt')
#     print(data)
