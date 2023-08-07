import numpy as np
# import matplotlib
# import matplotlib.pyplot as plt
import open3d as o3d
import cv2
import os
import shutil
import sys
import copy


if __name__ == '__main__':
    file_name_list = os.listdir('/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/cam0_rgb/')
    # file_name_list = os.listdir('/home/jyh/STC/20230628_CJ/Kinect/cam0_rgb_filtered/')
    checked_list = os.listdir('/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/cam0_rgb_filtered/')

    file_name_list.sort()
    checked_list.sort()

    if checked_list:
        latest_checked = checked_list[-1]
    else:
        latest_checked = None

    print(latest_checked)
    date_list = []

    print(len(date_list))

    copied_file_list = copy.deepcopy(file_name_list)

    for file_name in file_name_list:
        if(latest_checked == None):
            break
        
        copied_file_list.pop(0)

        if (file_name == latest_checked):
            break

    for file_name in copied_file_list:
        if(file_name[-1] == 'g'):
            date_list.append(file_name[9:-4])

    print(len(date_list))

    # RGB camera intrinsic Parameters:
    intrinsics = [963.1787109375, 0.0, 1020.224365234375, 0.0, 962.905517578125, 779.256591796875, 0.0, 0.0, 1.0]
    FX_RGB = intrinsics[0]
    FY_RGB = intrinsics[4]
    CX_RGB = intrinsics[2]
    CY_RGB = intrinsics[5]

    # Rotation matrix:
    R = -np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    # Translation vector:
    T = np.array([0.0, 0.0, 0.0])

    remains = len(date_list)

    try:
        for date_name in date_list:
            # plt.show()

            DATE = date_name
            RGB_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/cam0_rgb/cam0_rgb_{}.png'.format(DATE)
            RGB_SAVE_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/cam0_rgb_filtered/cam0_rgb_{}.png'.format(DATE)
            RGB_SAVE_PATH2 = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/cam0_rgb_filtered2/cam0_rgb_{}.png'.format(DATE)
            DEPTH_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/cam0_depth_to_rgb/cam0_depth_to_rgb_{}.png'.format(DATE)
            PCD_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230628/point_cloud/rgb_pointcloud_{}.pcd'.format(DATE)

            depth_image = cv2.imread(DEPTH_PATH, cv2.IMREAD_UNCHANGED)
            rgb_image = cv2.imread(RGB_PATH)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            print("Date : ", date_name, DEPTH_PATH)
            print(f"Image resolution: {depth_image.shape}")
            print(f"Data type: {depth_image.dtype}")
            print(f"Min value: {np.min(depth_image)}")
            print(f"Max value: {np.max(depth_image)}")
            print(f"Remains: {remains}")
            remains = remains - 1

            # Display depth and grayscale image:
            fig = plt.figure()
            axs = fig.add_subplot(1,1,1)
            # axs[0].imshow(depth_image, cmap="gray")
            # axs[0].set_title('Depth image')
            axs.imshow(depth_image)
            axs.set_title('RGB image')
            plt.draw()
            plt.pause(0.001)

            # compute point cloud:
            # Both images has the same resolution
            height, width = depth_image.shape

            depth_image = cv2.imread(DEPTH_PATH, cv2.IMREAD_UNCHANGED)
            rgb_image = cv2.imread(RGB_PATH)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            
            # rgb_image = o3d.geometry.Image(rgb_image)
            # depth_image = o3d.geometry.Image(depth_image)

            # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_image, depth_image, convert_rgb_to_intensity = False)
            # intrinsics = o3d.camera.PinholeCameraIntrinsic(width=width,height=height,fx=FX_RGB,fy=FY_RGB,cx=CX_RGB,cy=CY_RGB)
            # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)

            # flip the orientation, so it looks upright, not upside-down
            # pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])


            # o3d.visualization.draw_geometries([pcd])
            #plt.close('all')

            save_rgb = input()
            
            if save_rgb == 'p':
                shutil.copyfile(RGB_PATH, RGB_SAVE_PATH)
                shutil.copyfile(RGB_PATH, RGB_SAVE_PATH2)
            elif save_rgb == 'o':
                shutil.copyfile(RGB_PATH, RGB_SAVE_PATH)

            plt.close('all')

    except KeyboardInterrupt:
        sys.exit()
