#!/usr/bin/env python

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs

class PointCloudFusion:
    def __init__(self):
        rospy.init_node('point_cloud_fusion', anonymous=True)

        self.pcd1 = None
        self.pcd2 = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('/realsense_back_downsampled', PointCloud2, self.cloud1_callback)
        rospy.Subscriber('/realsense_front_downsampled', PointCloud2, self.cloud2_callback)
        #rospy.Subscriber('/realsense_back', PointCloud2, self.cloud1_callback)
        #rospy.Subscriber('/realsense_front', PointCloud2, self.cloud2_callback)
        self.pub = rospy.Publisher('/nube_repub', PointCloud2, queue_size=1)

    def cloud1_callback(self, msg):
        self.pcd1 = msg

    def cloud2_callback(self, msg):
        self.pcd2 = msg

    def convert_ros_to_pcl(self, ros_cloud):
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        return pcl_data

    def convert_pcl_to_ros(self, pcl_cloud, frame_id):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        ros_cloud = pc2.create_cloud_xyz32(header, pcl_cloud.to_list())
        return ros_cloud

    def fuse_clouds(self):
        if self.pcd1 is not None and self.pcd2 is not None:
            try:
                transform = self.tf_buffer.lookup_transform(self.pcd1.header.frame_id,
                                                            'realsense_435',
                                                            rospy.Time(0),
                                                            rospy.Duration(10.0))
                transformed_pcd2 = tf2_sensor_msgs.do_transform_cloud(self.pcd2, transform)

                pcl1 = self.convert_ros_to_pcl(self.pcd1)
                pcl2 = self.convert_ros_to_pcl(transformed_pcd2)

                # Convert PCL point clouds to numpy arrays for concatenation
                pcl1_np = np.asarray(pcl1.to_array())
                pcl2_np = np.asarray(pcl2.to_array())

                combined_np = np.vstack((pcl1_np, pcl2_np))

                # Convert the combined numpy array back to PCL point cloud
                combined_pcl = pcl.PointCloud()
                combined_pcl.from_array(combined_np.astype(np.float32))

                #ros_cloud = self.convert_pcl_to_ros(combined_pcl, self.pcd1.header.frame_id)
                ros_cloud = self.convert_pcl_to_ros(combined_pcl, "realsense_502")
                self.pub.publish(ros_cloud)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Transform not available")

    def run(self):
        rate = rospy.Rate(60)  # 10 Hz
        while not rospy.is_shutdown():
            self.fuse_clouds()
            rate.sleep()

if __name__ == '__main__':
    fusion_node = PointCloudFusion()
    fusion_node.run()
