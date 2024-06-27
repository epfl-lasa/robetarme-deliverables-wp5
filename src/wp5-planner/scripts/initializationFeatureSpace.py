import open3d as o3d
import numpy as np
import rospkg

def main():
    # Initialize the ROS package manager
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('wp5_planner')
    data_path = package_path + '/data'
    
    name_file = 'pointcloud_target_transformed'
    path_file = data_path + '/pointclouds/' + name_file + '.ply'

    # Step 1: Read and preprocess point cloud
    pcd = o3d.io.read_point_cloud(path_file)
    pcd = pcd.voxel_down_sample(voxel_size=0.01)

    # Step 2: Remove statistical outliers
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Step 3: Estimate normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(100)

    # Step 4: Create mesh using Poisson reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)

    # Step 5: Optional: Smooth the mesh
    mesh = mesh.filter_smooth_taubin(number_of_iterations=10, lambda_filter=0.5, mu=-0.53)



      # Step 7: Check if the mesh is manifold after cleaning
    if not mesh.is_edge_manifold(allow_boundary_edges=True):
        print("Warning: The mesh is still non-manifold after cleaning.")


    # Step 7: Density-based filtering

    vertices_to_remove = densities < np.quantile(densities, 0.05)  # Remove vertices with densities in the bottom 5%
    mesh.remove_vertices_by_mask(vertices_to_remove)

    # Step 8: Ensure the mesh has vertex normals and compute them
    mesh.compute_vertex_normals()

    # Step 9: Crop the mesh to a tight bounding box
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh = mesh.crop(bbox)


    # Step 6: Clean the mesh
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_degenerate_triangles()

    # Step 10: Optional: Further smooth the mesh
    mesh = mesh.filter_smooth_simple(number_of_iterations=3)


    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=500)

    # Step 11: Save the final mesh to a file
    output_mesh_file = data_path + '/meshes/' + name_file + '.obj'
    o3d.io.write_triangle_mesh(output_mesh_file, mesh)

    # Create coordinate axes
    coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    # Optional: Visualize the results with coordinate axes
    o3d.visualization.draw_geometries(
        [pcd, mesh, coord_axes], 
        window_name='Poisson Mesh with Coordinate Axes', 
        width=800, height=600, 
        point_show_normal=True, 
        mesh_show_wireframe=False
    )

if __name__ == '__main__':
    main()