#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import copy
import rospkg

class PointCloudDifference:
    def __init__(self):
        rospy.init_node('pointcloud_diff', anonymous=True)
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()

        # Get the path of a specific ROS package
        package_path = rospack.get_path('wp5_perception')
        
        self.loaded_cloud1 = o3d.io.read_point_cloud(package_path + "/data/pointclouds/transformed_pointcloud1.ply")
        self.loaded_cloud2 = o3d.io.read_point_cloud(package_path + "/data/pointclouds/transformed_pointcloud2.ply")

        # Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='PointCloud Difference', width=800, height=600)
        self.vis.add_geometry(self.loaded_cloud)

    def pointcloud_callback(self, msg):
        pc = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        received_cloud = o3d.geometry.PointCloud()
        received_cloud.points = o3d.utility.Vector3dVector(np.array(pc))

        diff_cloud = self.compute_difference(self.loaded_cloud, received_cloud)
        
        # Check if the difference cloud is empty
        if len(diff_cloud.points) == 0:
            rospy.logwarn("Difference cloud is empty, cannot compute boundary.")
            return
        
        boundary_cloud, triangles = self.extract_boundary(diff_cloud)

        self.update_visualization(diff_cloud, boundary_cloud, triangles)


    def compute_difference(self, cloud1, cloud2):
        distances = cloud1.compute_point_cloud_distance(cloud2)
        difference = cloud1.select_by_index([i for i, d in enumerate(distances) if d > 0.05])  # threshold can be adjusted
        return difference

    def extract_boundary(self, cloud):
        hull, indices = cloud.compute_convex_hull()
        triangles = np.asarray(hull.triangles)
        boundary_cloud = cloud.select_by_index(indices)
        return boundary_cloud, triangles

    def update_visualization(self, diff_cloud, boundary_cloud, triangles):
        self.vis.clear_geometries()

        # Paint colors
        diff_cloud.paint_uniform_color([1, 0, 0])  # Red color for difference cloud
        boundary_cloud.paint_uniform_color([0, 1, 0])  # Green color for boundary cloud

        # Add geometries to the scene
        self.vis.add_geometry(diff_cloud)
        self.vis.add_geometry(boundary_cloud)

        # Create LineSet for visualization of the triangles
        lines = []
        for tri in triangles:
            lines.append([tri[0], tri[1]])
            lines.append([tri[1], tri[2]])
            lines.append([tri[2], tri[0]])

        line_set = o3d.geometry.LineSet(
            points=boundary_cloud.points,
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set.paint_uniform_color([0, 0, 1])  # Blue color for lines

        # Add LineSet to the scene
        self.vis.add_geometry(line_set)

        # Update renderer
        self.vis.poll_events()
        self.vis.update_renderer()


if __name__ == '__main__':
    try:
        PointCloudDifference()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
