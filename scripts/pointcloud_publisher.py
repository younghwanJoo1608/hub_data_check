#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import cv2
import os
import shutil
import sys
import copy
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import * # convert float to uint32
import struct

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="cam0_rgb_camera_link"):
    
    pcl_msg = PointCloud2()
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    pcl_msg.header = header

    point_data = []
    
    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)

    fields=FIELDS_XYZRGB
    # -- Change rgb color from "three float" to "one 24-byte int"
    # 0x00FFFFFF is white, 0x00000000 is black.
    colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
    # print(points)
    # colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
    # cloud_data = [tuple((*p, c)) for p, c in zip(points, colors)]

    for i in range(len(points)):
        r = int(colors[i][0])
        g = int(colors[i][1])
        b = int(colors[i][2])
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [points[i][0], points[i][1], points[i][2], rgb]
        point_data.append(pt)
        
    # create ros_cloud
    return pc2.create_cloud(header, fields, point_data)


if __name__ == '__main__':
    rospy.init_node('pointcloud_publisher', anonymous=True)

    # RGB camera intrinsic Parameters:
    intrinsics = [982.9055786132812, 0.0, 1025.0330810546875, 0.0, 982.625, 781.5172729492188, 0.0, 0.0, 1.0]
    FX_RGB = intrinsics[0]
    FY_RGB = intrinsics[4]
    CX_RGB = intrinsics[2]
    CY_RGB = intrinsics[5]

    # Rotation matrix:
    R = -np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    # Translation vector:
    T = np.array([0.0, 0.0, 0.0])

    DATE = rospy.get_param('/pointcloud_publisher/date')
    FILE_NAME = rospy.get_param('/pointcloud_publisher/file_name')

    RGB_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/{0}/cam0_rgb/cam0_rgb_{1}.png'.format(DATE, FILE_NAME)
    DEPTH_PATH = '/media/jyh/Extreme SSD/unloading/CJHub/Kinect/{0}/cam0_depth_to_rgb/cam0_depth_to_rgb_{1}.png'.format(DATE, FILE_NAME)

    depth_image = cv2.imread(DEPTH_PATH, cv2.IMREAD_UNCHANGED)
    rgb_image = cv2.imread(RGB_PATH)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

    # compute point cloud:
    # Both images has the same resolution
    height, width = depth_image.shape

    depth_image = cv2.imread(DEPTH_PATH, cv2.IMREAD_UNCHANGED)
    rgb_image = cv2.imread(RGB_PATH)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    
    rgb_image = o3d.geometry.Image(rgb_image)
    depth_image = o3d.geometry.Image(depth_image)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_image, depth_image, convert_rgb_to_intensity = False)
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width=width,height=height,fx=FX_RGB,fy=FY_RGB,cx=CX_RGB,cy=CY_RGB)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)

    # # flip the orientation, so it looks upright, not upside-down
    # pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

    ros_cloud = convertCloudFromOpen3dToRos(pcd)
    
    pub = rospy.Publisher("/cam0/k4a/points2", PointCloud2, queue_size=1)
    
    while not rospy.is_shutdown():
        ros_cloud.header.stamp = rospy.Time.now()
        pub.publish(ros_cloud)
        rospy.sleep(1.0)
    