import numpy as np
import cv2
import os
import shutil
import sys
import copy


if __name__ == '__main__':
    day_list = [230621, 230622, 230628, 230629]

    SAVE_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect'

    for day_name in day_list:
        
        DAY = day_name
        DAY_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/{}'.format(DAY)

        RGB_FILTERED_PATH = DAY_PATH + '/cam0_rgb_filtered/'
        RGB_FILTERED_PATH2 = DAY_PATH + '/cam0_rgb_filtered2/'
        RGB_FILTERED_PATH3 = DAY_PATH + '/cam0_rgb_filtered3/'
        RGB_FILTERED_PATH4 = DAY_PATH + '/cam0_rgb_filtered4/'

        file_name_list = os.listdir(RGB_FILTERED_PATH)
        file_name_list2 = os.listdir(RGB_FILTERED_PATH2)
        file_name_list3 = os.listdir(RGB_FILTERED_PATH3)
        file_name_list4 = os.listdir(RGB_FILTERED_PATH4)

        file_name_list.sort()
        file_name_list2.sort()
        file_name_list3.sort()
        file_name_list4.sort()

        with open(SAVE_PATH + '/rgb_with_blue_with_workers.txt', 'a') as f:
            for file_name in file_name_list:
                if(file_name[-1] == 'g'):
                    f.write(file_name[9:-4] + '\n')

        with open(SAVE_PATH + '/rgb_without_blue_with_workers.txt', 'a') as f:
            for file_name in file_name_list2:
                if(file_name[-1] == 'g'):
                    f.write(file_name[9:-4] + '\n')

        with open(SAVE_PATH + '/rgb_with_blue_without_workers.txt', 'a') as f:
            for file_name in file_name_list3:
                if(file_name[-1] == 'g'):
                    f.write(file_name[9:-4] + '\n')

        with open(SAVE_PATH + '/rgb_without_blue_without_workers.txt', 'a') as f:
            for file_name in file_name_list4:
                if(file_name[-1] == 'g'):
                    f.write(file_name[9:-4] + '\n')