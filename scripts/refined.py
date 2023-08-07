import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import cv2
import os
import shutil
import sys
import copy


if __name__ == '__main__':
    file_path = "/home/jyh/STC/refined_image_pool_20221117.txt"

    with open(file_path) as f:
        lines = f.read().splitlines()

    for i in range(len(lines)):
        lines[i] = lines[i][10:] + "\n"

    with open("/home/jyh/STC/jyh.txt", "w") as file:
        file.writelines(lines)
        file.close()

    day7 = [[],[]]
    day8 = [[],[]]
    day9 = [[],[]]

    for i in range(len(lines)):
        ## 221107
        if lines[i][7] == '7':
            ## NUC1
            if lines[i][15] == '1':
                day7[0].append(lines[i][-37:-5] + "\n")
            ## NUC2
            else:
                day7[1].append(lines[i][-37:-5] + "\n")

        ## 221108
        if lines[i][7] == '8':
            ## NUC1
            if lines[i][15] == '1':
                day8[0].append(lines[i][-37:-5] + "\n")
            ## NUC2
            else:
                day8[1].append(lines[i][-37:-5] + "\n")

        ## 221109
        if lines[i][7] == '9':
            ## NUC1
            if lines[i][15] == '1':
                day9[0].append(lines[i][-37:-5] + "\n")
            ## NUC2
            else:
                day9[1].append(lines[i][-37:-5] + "\n")

    print(day7[0][0])

    with open("/home/jyh/STC/221107_NUC1.txt", "w") as file:
        file.writelines(day7[0])
        file.close()

    with open("/home/jyh/STC/221107_NUC2.txt", "w") as file:
        file.writelines(day7[1])
        file.close()

    with open("/home/jyh/STC/221108_NUC1.txt", "w") as file:
        file.writelines(day8[0])
        file.close()

    with open("/home/jyh/STC/221108_NUC2.txt", "w") as file:
        file.writelines(day8[1])
        file.close()

    with open("/home/jyh/STC/221109_NUC1.txt", "w") as file:
        file.writelines(day9[0])
        file.close()

    with open("/home/jyh/STC/221109_NUC2.txt", "w") as file:
        file.writelines(day9[1])
        file.close()