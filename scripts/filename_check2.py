import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import cv2
import os
import shutil
import sys
import copy

if __name__ == '__main__':

    file_name_list = os.listdir('/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/point_cloud')
    date_list = []
    for file_name in file_name_list:
        if(file_name[-1] == 'd'):
            date_list.append(file_name[15:-4])

    date_list.sort()

    i = 0

    try:
        for DATE in date_list:

            # RGB_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/cam0_rgb/cam0_rgb_{}.png'.format(DATE)
            # RGB_SAVE_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/cam0_rgb/cam0_rgb_{}.png'.format(DATE)
            # DEPTH_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/cam0_depth/cam0_depth_{}.png'.format(DATE)
            # DEPTH_SAVE_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/cam0_depth/cam0_depth_{}.png'.format(DATE)
            # DEPTH_TO_RGB_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/cam0_depth_to_rgb/cam0_depth_to_rgb_{}.png'.format(DATE)
            # DEPTH_TO_RGB_SAVE_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/cam0_depth_to_rgb/cam0_depth_to_rgb_{}.png'.format(DATE)
            # RGB_TO_DEPTH_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/cam0_rgb_to_depth/cam0_rgb_to_depth_{}.png'.format(DATE)
            # RGB_TO_DEPTH_SAVE_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/cam0_rgb_to_depth/cam0_rgb_to_depth_{}.png'.format(DATE)
            # IR_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/cam0_ir/cam0_ir_{}.png'.format(DATE)
            # IR_SAVE_PATH = '/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/cam0_ir/cam0_ir_{}.png'.format(DATE)

            # shutil.copyfile(RGB_PATH, RGB_SAVE_PATH)
            # shutil.copyfile(DEPTH_PATH, DEPTH_SAVE_PATH)
            # shutil.copyfile(DEPTH_TO_RGB_PATH, DEPTH_TO_RGB_SAVE_PATH)
            # shutil.copyfile(RGB_TO_DEPTH_PATH, RGB_TO_DEPTH_SAVE_PATH)
            # shutil.copyfile(IR_PATH, IR_SAVE_PATH)

            line = DATE + "\n"

            with open("/home/jyh/STC/20221108_CJ/Nuc1/Nuc1/221108_Nuc1.txt", "a") as file:
                file.writelines(line)
                file.close()
    except KeyboardInterrupt:
        sys.exit()
