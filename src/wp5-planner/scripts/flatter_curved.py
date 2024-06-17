import open3d as o3d

# Load point cloud
point_cloud = o3d.io.read_point_cloud("../data/pointclouds/transformed_pointcloud_curved.ply")

# Noise Removal
# Perform statistical outlier removal
cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
point_cloud_clean = point_cloud.select_by_index(ind)

# Normal Estimation
# Estimate normals
point_cloud_clean.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Surface Reconstruction
# Poisson surface reconstruction
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud_clean, depth=9)

# Smoothing
# Apply Laplacian smoothing
mesh.filter_smooth_laplacian(3)

# Convert mesh to point cloud
point_cloud_reconstructed = mesh.sample_points_poisson_disk(number_of_points=len(mesh.vertices))

# Save point cloud
o3d.io.write_point_cloud("../data/pointclouds/transformed_pointcloud_curved_post.ply", point_cloud_reconstructed, write_ascii=True)

# path_file = '../data/meshes/pointcloud_target_transformed.obj'
# o3d.io.write_triangle_mesh(path_file, mesh)