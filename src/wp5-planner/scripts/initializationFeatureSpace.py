#initializationFeatureSpace
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time
from sklearn.metrics import mean_squared_error
from math import sqrt
import sys
import rospkg


def reconstruct_surface(pcd, method='alpha_shapes', **kwargs):
    """
    Reconstructs a surface from a point cloud.

    Parameters:
    - pcd: open3d.geometry.PointCloud
        The input point cloud.
    - method: str
        The surface reconstruction method to use. Options are 'alpha_shapes', 'ball_pivoting', and 'poisson'.
    - kwargs: dict
        Method-specific parameters.

    Returns:
    - mesh: open3d.geometry.TriangleMesh
        The reconstructed surface.
    """
    if method == 'alpha_shapes':
        alpha = kwargs.get('alpha', 0.03)
        if 'tetra_mesh' in kwargs and 'pt_map' in kwargs:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha, kwargs['tetra_mesh'], kwargs['pt_map'])
        else:
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
            
    elif method == 'ball_pivoting':
        radii = kwargs.get('radii', [0.005, 0.01, 0.02, 0.04])
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
        
    elif method == 'poisson':
        depth = kwargs.get('depth', 9)
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth)
        
        # Optional: Visualize or use the densities for further processing, e.g., removing low-density vertices.
        # This step is skipped in this example to keep the function focused on reconstruction.
        
    else:
        raise ValueError("Unsupported method. Choose from 'alpha_shapes', 'ball_pivoting', or 'poisson'.")
    
    mesh.compute_vertex_normals()  # Compute normals for visualization
    return mesh

def visualize_with_timeout(geometry, timeout=5, use_time_count=True):
    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(geometry)
    vis.get_render_option().point_show_normal = True
    
    if use_time_count:
        # Run the visualizer for a set amount of time (timeout) and then close
        for _ in range(int(timeout * 20)):  # Assuming 20 iterations per second as an example
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.05)  # Adjust sleep time as needed
    else:
        # Keep the window open until the user closes it. This will block the script from continuing until the window is closed.
        print("Close the visualization window to continue.")
        vis.run()  # This starts the event loop and waits until the window is closed

    vis.destroy_window()


def estimate_point_density(pcd):
    # Estimate average distance to the nearest neighbor
    distances = pcd.compute_nearest_neighbor_distance()
    avg_distance = np.mean(distances)
    max_distance = max(distances)
    min_distance = min(distances)
    return avg_distance, max_distance, min_distance

def adapt_radii_based_on_density(pcd):
    num_radii=4
    avg_distance, max_distance, min_distance = estimate_point_density(pcd)
    # Adapt the scale factor based on average, min and max distance
    # radii = np.linspace(min_distance*0.9, max_distance*1.1, num_radii).tolist()
    radii = [min_distance*0.8,min_distance*0.8*2, min_distance*0.8*4, min_distance*0.8*8]
    radii = [round(r, 6) for r in radii]  # Apply round to each element in radii
    # print(f"Radii: {radii}")  # Print the radii
    return radii

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],)


def evaluate_mesh_pointcloud(mesh, pointcloud):
    # Convert mesh to a point cloud representation
    mesh_points = np.asarray(mesh.vertices)
    mesh_pc = o3d.geometry.PointCloud()
    mesh_pc.points = o3d.utility.Vector3dVector(mesh_points)

    # Compute distances from each point in the point cloud to the surface of the mesh
    distances = pointcloud.compute_point_cloud_distance(mesh_pc)

    # Compute the average distance and standard deviation
    avg_distance = np.mean(distances)
    std_distance = np.std(distances)

    return avg_distance, std_distance

def test_remove_statistical_outlier(pcd, nb_neighbors_list, std_ratio_list):
    results = []

    for nb_neighbors in nb_neighbors_list:
        for std_ratio in std_ratio_list:
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
            num_points_removed = len(pcd.points) - len(cl.points)
            results.append((nb_neighbors, std_ratio, num_points_removed, cl, ind))

    # Calculate the Z-scores
    num_points_removed_list = [result[2] for result in results]
    mean = np.mean(num_points_removed_list)
    std_dev = np.std(num_points_removed_list)
    z_scores = [(result[0], result[1], (result[2] - mean) / std_dev, result[3], result[4]) for result in results]

    # Find the result with the Z-score closest to 0
    best_result = min(z_scores, key=lambda x: abs(x[2]))

    return best_result[3], best_result[4]  # Return only cl and ind


def main()->bool:
    # Initialize the ROS package manager
    rospack = rospkg.RosPack()

    package_path = rospack.get_path('wp5_planner')
    data_path = package_path + '/data'

    name_file = 'pointcloud_target_transformed'

    path_file = data_path + '/pointclouds/' + name_file + '.ply'

    pcd = o3d.io.read_point_cloud(path_file)
    pcd = pcd.voxel_down_sample(voxel_size=0.01)

    nb_neighbors_list = [10, 20, 30, 40, 50]  # List of nb_neighbors values to test
    std_ratio_list = [1.0, 2.0, 3.0]  # List of std_ratio values to test

    # Remove statistical outliers
    pcd, ind = best_result = test_remove_statistical_outlier(pcd, nb_neighbors_list, std_ratio_list)


    # estimate normals
    # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))  # invalidate existing normals
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(100)

    radii = adapt_radii_based_on_density(pcd)

    # # Define the methods and their parameters
    # method = {'name': 'ball_pivoting', 'params': {'scale_factor': [5, 10, 11, 12, 13]}}

    # # Placeholder for the best method and parameters
    # best_method = None
    # best_params = None
    # best_mesh = None
    # best_std = float('inf')

    # # Loop over the parameters
    # for param_key, param_values in method['params'].items():
    #     for param_value in param_values:
    #         original_radii = adapt_radii_based_on_density(pcd)
    #         radii = [r * param_value for r in original_radii]  # Scale the radii
    #         mesh = reconstruct_surface(pcd, method=method['name'], radii=radii)
    #         avg, std = evaluate_mesh_pointcloud(mesh, pcd)
    #         # Update the best method and parameters if this RMSE is lower
    #         if std < best_std:
    #             best_method = method['name']
    #             best_params = {param_key: param_value}
    #             best_std = std
    #             best_mesh = mesh

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
    # Step 1: Clean the mesh
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    # Ensure the mesh has vertex normals
    mesh.compute_vertex_normals()
    bbox = mesh.get_axis_aligned_bounding_box()
    mesh_poisson = mesh.crop(bbox)
    mesh_smooth = mesh_poisson.filter_smooth_simple(number_of_iterations=3)

    path_file = data_path + '/meshes/' + name_file + '.obj'
    o3d.io.write_triangle_mesh(path_file, mesh_smooth)


    return True





if __name__ == '__main__':
    main()
