#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

bridge = CvBridge()
camera_info_received = False
camera_info = None
depth_image = None

def info_callback(info_msg):
    global camera_info, camera_info_received
    camera_info = info_msg
    camera_info_received = True
    rospy.loginfo("Camera info received, unsubscribing from /camera1/depth/camera_info")
    camera_info_sub.unregister()  # Unsubscribe after receiving the camera info once

def depth_callback(depth_msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

def main():
    global depth_image, camera_info_received, camera_info_sub, pc_pub

    rospy.init_node('depth_to_pointcloud_front')
    
    rospy.Subscriber('/camera2/depth/image_rect_raw', Image, depth_callback)
    camera_info_sub = rospy.Subscriber('/camera2/depth/camera_info', CameraInfo, info_callback)
    
    pc_pub = rospy.Publisher('/realsense_front', PointCloud2, queue_size=1)

    rate = rospy.Rate(80)  # Adjust the rate as necessary
    while not rospy.is_shutdown():
        if camera_info_received and depth_image is not None:
            # Reduce the resolution of the depth image to half
            depth_image_resized = cv2.resize(depth_image, (depth_image.shape[1] // 3, depth_image.shape[0] // 3), interpolation=cv2.INTER_NEAREST)
            depth_array = np.array(depth_image_resized, dtype=np.float32)

            # Get camera intrinsic parameters
            fx = camera_info.K[0] / 2.0
            fy = camera_info.K[4] / 2.0
            cx = camera_info.K[2] / 2.0
            cy = camera_info.K[5] / 2.0

            height, width = depth_array.shape

            # Create a grid of (u, v) coordinates
            u = np.tile(np.arange(width), (height, 1))
            v = np.tile(np.arange(height).reshape(-1, 1), (1, width))

            # Calculate the corresponding (x, y, z) coordinates
            z = depth_array / 1000.0  # Assuming the depth is in millimeters
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            # Filter out zero depths
            valid = z > 0
            x = x[valid]
            y = y[valid]
            z = z[valid]

            # Stack the coordinates to form the point cloud
            points = np.stack((x, y, z), axis=-1)

            # Create PointCloud2 message
            header = camera_info.header
            header.frame_id = "realsense_435"
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            point_cloud_msg = pc2.create_cloud(header, fields, points)
            pc_pub.publish(point_cloud_msg)

        rate.sleep()

if __name__ == '__main__':
    main()

