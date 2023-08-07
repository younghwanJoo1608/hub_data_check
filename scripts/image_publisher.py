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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


if __name__ == '__main__':
    rospy.init_node('image_publisher', anonymous=True)
    br = CvBridge()

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

    rgb_pub = rospy.Publisher("/cam0/k4a/rgb/image_raw", Image, queue_size=10)
    depth_pub = rospy.Publisher("/cam0/k4a/depth_to_rgb/image_raw", Image, queue_size=10)
    
    rgb_msg = br.cv2_to_imgmsg(rgb_image, encoding='rgb8')
    depth_msg = br.cv2_to_imgmsg(depth_image, encoding='mono16')
    
    while not rospy.is_shutdown():
        rgb_msg.header.stamp = rospy.Time.now()
        depth_msg.header.stamp = rospy.Time.now()
        rgb_pub.publish(rgb_msg)
        depth_pub.publish(depth_msg)
        rospy.sleep(1.0)
    