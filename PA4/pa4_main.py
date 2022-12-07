import numpy as np

def main():
    A = []
    B = []

    for i in range(3):
        A.append([1,2,3])
        B.append([4,5,6])
    
    error = A[[j for j in range(len(A))]] - B[[j for j in range(len(B))]]

    print(error)

if __name__ == "__main__":
    main()