#!/usr/bin/env python

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
import numpy as np

def read_ply_and_publish(ply_file_path, topic_name):
    # Initialize ROS node
    rospy.init_node('ply_to_pointcloud2_node', anonymous=True)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # Publish rate in Hz

    # Read the PLY file using Open3D
    point_cloud = o3d.io.read_point_cloud(ply_file_path)
    points = np.asarray(point_cloud.points)

    # Create the PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"  # or any other frame you are using

    # Create the PointCloud2 message using create_cloud_xyz32
    point_cloud_msg = create_cloud_xyz32(header, points)

    while not rospy.is_shutdown():
        # Publish the PointCloud2 message
        pub.publish(point_cloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # ply_file_path = "../data/pointclouds/combined_square_noisy_transformed.ply"  # Replace with your PLY file path
        # ply_file_path = "../data/pointclouds/combined_square_inverted_noisy_transformed.ply"  # Replace with your PLY file path
        # ply_file_path = "../data/pointclouds/half_cylinder_noisy_transformed.ply"  # Replace with your PLY file path
        # ply_file_path = "../data/pointclouds/half_cylinder_noisy_inverted_transformed.ply"  # Replace with your PLY file path
        ply_file_path = "../data/pointclouds/half_sphere_noisy_transformed.ply"  # Replace with your PLY file path
        # ply_file_path = "../data/pointclouds/half_sphere_noisy_inverted_transformed.ply"  # Replace with your PLY file path
        # ply_file_path = "../data/pointclouds/square_noisy_transformed.ply"  # Replace with your PLY file path





        topic_name = "/toydata/pointcloud2"  # Replace with your desired topic name
        read_ply_and_publish(ply_file_path, topic_name)
    except rospy.ROSInterruptException:
        pass
