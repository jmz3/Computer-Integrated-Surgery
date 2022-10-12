from ast import main
import numpy as np
import pandas as pd

def load_txt_data(file_name):
    data = np.loadtxt(file_name,skiprows=1,delimiter=',')
    data_info = np.loadtxt(file_name,max_rows=1,dtype=str,delimiter=',')
    return data,data_info



# if __name__ == "__main__":
#     data = load_txt_data('PA1 Student Data/pa1-debug-a-calbody.txt')
#     print(data)
