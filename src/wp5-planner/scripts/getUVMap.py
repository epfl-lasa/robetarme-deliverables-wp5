import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import rospkg


def main():
    # Initialize the ROS package manager
    rospack = rospkg.RosPack()

    package_path = rospack.get_path('wp5_planner')
    data_path = package_path + '/data'
    name_file = 'pointcloud_target_transformed'

    mesh_file_path = data_path + '/meshes/' + name_file + '.obj'
    mesh = o3d.io.read_triangle_mesh(mesh_file_path)

    mesh.compute_vertex_normals()
    normals_vector = np.asarray(mesh.vertex_normals)

    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    max_stretch = 0.5  # Maximum stretch allowed for UV mapping, adjust as needed
    gutter = 1
    size = 512  # Target texture size, adjust as needed
    mesh.compute_uvatlas(size=size, gutter=gutter, max_stretch=max_stretch)
    vertices_3D = mesh.vertex['positions'].numpy()
    uv_coords = mesh.triangle['texture_uvs'].numpy()

    # Number of 3D mesh vertices
    num_vertices = mesh.vertex['positions'].shape[0]
    num_triangles = mesh.triangle['indices'].shape[0]
    num_uv_coords = mesh.triangle['texture_uvs'].shape[0]
    #--- from mesh, get point by triangles, and the point on triangles are the same as the point on the mesh 
    #--- with right identity
    triangle_index = mesh.triangle['indices'].numpy()
    #--- add the triangle_index in the each 2D point of uv_coords as the index
    #--- Reshape triangle_index to match the shape of uv_coords
    triangle_index_reshaped = triangle_index.reshape(num_triangles, 3, 1)
    triangle_index_reshaped = triangle_index_reshaped.astype(int)
    #--- Add triangle_index to uv_coords
    # uv_coords_index = np.dstack((triangle_index_reshaped,uv_coords))
    uv_coords_index = np.dstack((triangle_index_reshaped,uv_coords))  #--- test with triangle
    #--- Reshape uv_coords_index into a two-dimensional array
    uv_coords_index_reshaped = uv_coords_index.reshape(-1, uv_coords_index.shape[-1])

    Uv_path_reshaped = data_path + '/UVmap/uv_coords_index_reshaped.txt'
    np.savetxt(Uv_path_reshaped, uv_coords_index_reshaped, fmt='%f', delimiter=' ')

    #--- Remove duplicate rows
    uv_coords_index_unique = np.unique(uv_coords_index_reshaped, axis=0)
    Uv_path_unique = data_path + '/UVmap/uv_coords_index_unique.txt'

    np.savetxt(Uv_path_unique, uv_coords_index_unique, fmt='%f', delimiter=' ')

    # Get the first column
    first_column = uv_coords_index_unique[:, 0]
    # Find unique values and their counts
    unique_values, counts = np.unique(first_column, return_counts=True)
    # Filter for values that have a count greater than 1
    repeated_values = unique_values[counts > 1]

    # Find the rows that contain the repeated values
    rows_with_repeated_values = np.isin(first_column, repeated_values)
    # Output the rows
    repeated_rows = uv_coords_index_unique[rows_with_repeated_values]
    # Find unique values and the indices of their first occurrence
    unique_values, indices = np.unique(first_column, return_index=True)
    # Select the rows with unique values in the first column
    unique_rows = uv_coords_index_unique[indices]
    uv_coords_index_unique=unique_rows


    if uv_coords_index_unique.shape[0] != vertices_3D.shape[0]:
        print(f"\033[91mERROR: uv_coords_index_unique has different number of vertices_3D\033[0m")
    else:
        #--- put the 3D vertices in the uv_coords_index_unique
        print(f"\033[91m SUCCESS !!! \033[0m")
        uv_coords_index_unique= np.hstack((uv_coords_index_unique,vertices_3D))
        unique_uv_coords=uv_coords_index_unique[:,1:]
            # Plot the unique UV coordinates as points
        plt.figure(figsize=(5, 5))
        plt.scatter(uv_coords_index_unique[:, 1], uv_coords_index_unique[:, 2], s=1)  # s controls the size of the points
        plt.title("UV Map")
        plt.xlabel("U")
        plt.ylabel("V")
        plt.gca().set_aspect('equal', adjustable='box')  # Set equal aspect ratio
        plt.show()

    Uv_path_reshaped = data_path + '/UVmap'

    np.savetxt(Uv_path_reshaped + "/uv_points_" +name_file+ ".txt", unique_uv_coords, fmt='%f', delimiter=' ')
    np.savetxt(Uv_path_reshaped +"/uv_map_" +name_file+ ".txt", uv_coords_index_unique, fmt='%f', delimiter=' ')
    np.savetxt(Uv_path_reshaped +"/normvector_" +name_file+ ".txt", normals_vector, fmt='%f', delimiter=' ')




if __name__ == '__main__':
    main()
