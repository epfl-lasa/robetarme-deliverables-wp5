#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import os
import rospkg
import time
import open3d as o3d
from std_msgs.msg import Header

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('pointcloud_transformer')  # Initialize ROS node

        # Initialize rospkg and get package path
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('wp5_perception')

        rospy.Subscriber('/stereo/clipped_point_cloud', PointCloud2, self.depth_image_callback)
        self.transformed_pc_pub = rospy.Publisher('/stereo/pointcloud2/points_transformed', PointCloud2, queue_size=100)

    def depth_image_callback(self, msg):
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        if len(points_list) == 0:
            rospy.logwarn("Received empty point cloud")
            return

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(np.array(points_list))
        
        # Load transformation matrix
        transformation_matrix_path = os.path.join(self.package_path, "data/matrix/Y1_matrices.npy")
        transformation_matrix = np.load(transformation_matrix_path)
        
        point_cloud_transformed = point_cloud.transform(transformation_matrix)

        points_transformed = np.asarray(point_cloud_transformed.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        point_cloud2_msg = pc2.create_cloud(header, fields, points_transformed)
        self.transformed_pc_pub.publish(point_cloud2_msg)


if __name__ == '__main__':
    pc_transformer = PointCloudTransformer()
    rospy.spin()  # Start ROS node and enter loop to process callbacks
