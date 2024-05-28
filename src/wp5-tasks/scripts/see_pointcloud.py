

#!/usr/bin/env python

import rospy
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
def read_ply(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)
    return points

def publish_pointcloud():
    rospy.init_node('ply_pointcloud_publisher', anonymous=True)
    pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    file_path = '../pointclouds/transformed_pointcloud.ply'
    points = read_ply(file_path)

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    header = std_msgs.msg.Header()
    header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        pc2_msg = pc2.create_cloud(header, fields, points)
        pub.publish(pc2_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pointcloud()
    except rospy.ROSInterruptException:
        pass
