import open3d as o3d
import numpy as np
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('wp5_perception')
# Load two point clouds
output_file_path1 = package_path + "/data/pointclouds/transformed_pointcloud1.ply"
output_file_path2 = package_path + "/data/pointclouds/transformed_pointcloud2.ply"

pcd1 = o3d.io.read_point_cloud(output_file_path1)
pcd2 = o3d.io.read_point_cloud(output_file_path2)

# Compute the distance of each point in pcd1 to pcd2
distances = pcd1.compute_point_cloud_distance(pcd2)
distances = np.asarray(distances)  # Convert to numpy array

# Create a new point cloud for the distances
diff_pcd = o3d.geometry.PointCloud()
diff_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd1.points)[distances > 0.])

# Visualize the difference
o3d.visualization.draw_geometries([diff_pcd])