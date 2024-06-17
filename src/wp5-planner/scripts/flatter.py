

import open3d as o3d
import numpy as np

def load_point_cloud(file_path):
    """Load the point cloud from a file."""
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def estimate_plane(point_cloud):
    """Estimate the best fitting plane using RANSAC."""
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.1,
                                                     ransac_n=3,
                                                     num_iterations=1000)
    [a, b, c, d] = plane_model
    return plane_model, inliers

def flatten_point_cloud(point_cloud, plane_model):
    """Flatten the point cloud by projecting it onto the estimated plane."""
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    
    points = np.asarray(point_cloud.points)
    centroid = np.mean(points, axis=0)
    
    # Center points around the centroid
    centered_points = points - centroid
    
    # Project points onto the plane
    flattened_points = centered_points - np.dot(centered_points, normal).reshape(-1, 1) * normal
    
    # Shift flattened points back to the original centroid
    flattened_points += centroid
    
    return flattened_points

def fill_holes(flattened_points):
    """Fill holes in the flattened point cloud using Poisson surface reconstruction."""
    # Create a PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(flattened_points)
    
    # Estimate normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    # Perform Poisson surface reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)
    
    # Sample points from the mesh to create a new point cloud
    filled_pcd = mesh.sample_points_poisson_disk(number_of_points=len(flattened_points))
    
    return filled_pcd, mesh

def visualize_point_clouds(original_pcd, filled_pcd):
    """Visualize the original and filled flattened point clouds in the same plot."""
    original_pcd.paint_uniform_color([1, 0, 0])  # Original point cloud in red
    filled_pcd.paint_uniform_color([0, 1, 0])  # Filled point cloud in green
    o3d.visualization.draw_geometries([original_pcd, filled_pcd])

def save_point_cloud(points, output_file):
    """Save the point cloud to a file."""
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    o3d.io.write_triangle_mesh(output_file, mesh)

# Main script
path_file = '../data/pointclouds/transformed_pointcloud_flat.ply'
output_file = "../data/pointclouds/transformed_pointcloud_flat_post.ply"  # Save as .ply file

# Load the original point cloud
original_pcd = load_point_cloud(path_file)

# Estimate the plane using the original point cloud
plane_model, inliers = estimate_plane(original_pcd)

# Flatten the original point cloud
flattened_points = flatten_point_cloud(original_pcd, plane_model)

# Fill holes in the flattened point cloud
# filled_pcd, mesh = fill_holes(flattened_points)


# Create an Open3D PointCloud object
pcd = o3d.geometry.PointCloud()

# Assign your point cloud data to the Open3D PointCloud object
pcd.points = o3d.utility.Vector3dVector(flattened_points)

# Save the PointCloud object to a .ply file
o3d.io.write_point_cloud(output_file, pcd, write_ascii=True)

# Visualize the original and filled flattened point clouds
visualize_point_clouds(original_pcd, pcd)


