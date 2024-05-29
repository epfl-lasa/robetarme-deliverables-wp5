import numpy as np
import open3d as o3d

# Step 1: Load point cloud data from .ply file
point_cloud = o3d.io.read_point_cloud("../data/pointclouds/pointcloud_target.ply")

# Step 2: Load transformation matrix from .npy file
transformation_matrix = np.load("../data/matrix/Y1_matrices.npy")
# Print the transformation matrix
print("Transformation Matrix:")
print(transformation_matrix)

# Step 3: Apply transformation matrix to point cloud
point_cloud_transformed = point_cloud.transform(transformation_matrix)

# # Step 4: Check visualization of transformed point cloud
# o3d.visualization.draw_geometries([point_cloud_transformed])

# Step 5: Save transformed point cloud data
output_file_path = "../data/pointclouds/pointcloud_target_transformed.ply"
success = o3d.io.write_point_cloud(output_file_path, point_cloud_transformed, write_ascii=True)
if success:
    print("Transformed point cloud saved successfully.")
else:
    print("Failed to save transformed point cloud.")

