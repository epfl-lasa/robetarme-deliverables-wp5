import numpy as np
import open3d as o3d
import math

def generate_square(side_length, num_points):
    x = np.random.uniform(-side_length / 2, side_length / 2, num_points)
    y = np.random.uniform(-side_length / 2, side_length / 2, num_points)
    z = np.zeros(num_points)
    return np.vstack((x, y, z)).T

def generate_half_cylinder(radius, height, num_points):
    theta = np.random.uniform(0, np.pi, num_points)
    z = np.random.uniform(-height / 2, height / 2, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return np.vstack((x, y, z)).T

def generate_half_sphere(radius, num_points):
    phi = np.random.uniform(0, np.pi / 2, num_points)  # Restrict to half sphere
    theta = np.random.uniform(0, 2 * np.pi, num_points)
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    return np.vstack((x, y, z)).T

def create_open3d_point_cloud(points):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def apply_transformation(points, translation, rotation):
    # Create a 4x4 transformation matrix
    T = np.eye(4)
    
    # Set the translation part
    T[:3, 3] = translation
    
    # Set the rotation part
    T[:3, :3] = rotation
    
    # Convert points to homogeneous coordinates
    ones = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack((points, ones))
    
    # Apply the transformation
    transformed_points = (T @ homogeneous_points.T).T
    
    return transformed_points[:, :3]

def add_noise(points, noise_level):
    noise = np.random.normal(0, noise_level, points.shape)
    noisy_points = points + noise
    return noisy_points

def rotation_matrix_x(angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])
def rotation_matrix_y(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

def rotation_matrix_z(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

# Example rotation matrices for 90 degrees rotation around x, y, and z axis
rotation_x = np.array([[1, 0, 0],
                       [0, np.cos(np.pi/2), -np.sin(np.pi/2)],
                       [0, np.sin(np.pi/2), np.cos(np.pi/2)]])
rotation_45_x = np.array([[1, 0, 0],
                       [0, np.cos(np.pi/4), -np.sin(np.pi/4)],
                       [0, np.sin(np.pi/4), np.cos(np.pi/4)]])
rotation_45_x_minus = np.array([[1, 0, 0],
                       [0, np.cos(np.pi/-4), -np.sin(np.pi/-4)],
                       [0, np.sin(np.pi/-4), np.cos(np.pi/-4)]])

rotation_y = np.array([[np.cos(np.pi/2), 0, np.sin(np.pi/2)],
                       [0, 1, 0],
                       [-np.sin(np.pi/2), 0, np.cos(np.pi/2)]])

rotation_z = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0],
                       [np.sin(np.pi/2), np.cos(np.pi/2), 0],
                       [0, 0, 1]])

rotation_45_z = np.array([[np.cos(np.pi/4), -np.sin(np.pi/4), 0],
                          [np.sin(np.pi/4), np.cos(np.pi/4), 0],
                          [0, 0, 1]])

no_rotation = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

# Example rotation matrix for 90 degrees rotation around y-axis
rotation_90_y = np.array([[np.cos(np.pi/2), 0, np.sin(np.pi/2)],
                          [0, 1, 0],
                          [-np.sin(np.pi/2), 0, np.cos(np.pi/2)]])

# Generate point clouds
side_length = 0.3# Generate point clouds
square_points = generate_square(side_length, 1000)
square_point = generate_square(2*side_length, 1000)
half_cylinder_points = generate_half_cylinder(0.6, 0.6, 1000)
half_cylinder_points_i = generate_half_cylinder(0.2, 0.2, 1000)
half_sphere_points = generate_half_sphere(0.2, 1000)

# Apply transformations: rotation to the second square
# Apply transformations: position second square to share an edge with the first
desired_angle = math.pi/6
translation = [0,(-side_length / 2)*math.cos(desired_angle), 0]
square_points1_transformed = apply_transformation(square_points, translation=translation, rotation=rotation_matrix_x(desired_angle))
translation = [0,(side_length / 2)*math.cos(desired_angle), 0]
square_points2_transformed = apply_transformation(square_points, translation=translation, rotation=rotation_matrix_x(-desired_angle))
combined_points = np.vstack((square_points1_transformed, square_points2_transformed))

# invert the direction of target
translation = [0, 0, 0]
combined_point_inverted = apply_transformation(combined_points, translation=translation, rotation=rotation_matrix_x(math.pi))
half_sphere_points_inverted = apply_transformation(half_sphere_points, translation=translation, rotation=rotation_matrix_x(math.pi))
half_cylinder_points_inverted = apply_transformation(half_cylinder_points_i, translation=translation, rotation=rotation_matrix_x(math.pi))


# Apply transformations: translationand rotation to bwe infront of the robot
square_points_transformed = apply_transformation(square_point, translation=[1, 0, 0.8], rotation=rotation_matrix_y(math.pi/2))
half_cylinder_points_transformed = apply_transformation(half_cylinder_points, translation=[0.5, 0, 0.8], rotation=rotation_matrix_z(-math.pi/2))
half_sphere_points_transformed = apply_transformation(half_sphere_points, translation=[1, 0, 0.8], rotation=rotation_matrix_y(math.pi/2))
combined_points_transformed = apply_transformation(combined_points, translation=[1, 0, 0.8], rotation=rotation_matrix_y(math.pi/2))

half_cylinder_points_inverted_transformed = apply_transformation(half_cylinder_points_inverted, translation=[0.7, 0, 0.8], rotation=rotation_matrix_z(-math.pi/2))
half_sphere_points_inverted_transformed = apply_transformation(half_sphere_points_inverted, translation=[1, 0, 0.8], rotation=rotation_matrix_y(math.pi/2))
combined_point_inverted_transformed = apply_transformation(combined_point_inverted, translation=[1, 0, 0.8], rotation=rotation_matrix_y(math.pi/2))

# Add noise to the transformed points
noise_level = 0.01  # Standard deviation of the Gaussian noise
square_points_noisy = add_noise(square_points_transformed, noise_level)
half_cylinder_points_noisy = add_noise(half_cylinder_points_transformed, noise_level)
half_sphere_points_noisy = add_noise(half_sphere_points_transformed, noise_level)
combined_points_noisy = add_noise(combined_points_transformed, noise_level)

half_cylinder_points_inverted_noisy = add_noise(half_cylinder_points_inverted_transformed, noise_level)
half_sphere_points_inverted_noisy = add_noise(half_sphere_points_inverted_transformed, noise_level)
combined_points_inverted_noisy = add_noise(combined_point_inverted_transformed, noise_level)

# Create Open3D point clouds
square_pcd = create_open3d_point_cloud(square_points_noisy)
half_cylinder_pcd = create_open3d_point_cloud(half_cylinder_points_noisy)
half_sphere_pcd = create_open3d_point_cloud(half_sphere_points_noisy)
combined_pcd = create_open3d_point_cloud(combined_points_noisy)
half_cylinder_pcd_inverted = create_open3d_point_cloud(half_cylinder_points_inverted_noisy)
half_sphere_pcd_inverted = create_open3d_point_cloud(half_sphere_points_inverted_noisy)
combined_pcd_inverted = create_open3d_point_cloud(combined_points_inverted_noisy)


# Save point clouds to PLY files
path = "../data/pointclouds/"
o3d.io.write_point_cloud(path + "square_noisy_transformed.ply", square_pcd,write_ascii=True)
o3d.io.write_point_cloud(path + "half_cylinder_noisy_transformed.ply", half_cylinder_pcd,write_ascii=True)
o3d.io.write_point_cloud(path + "half_sphere_noisy_transformed.ply", half_sphere_pcd,write_ascii=True)
o3d.io.write_point_cloud(path + "combined_square_noisy_transformed.ply", combined_pcd,write_ascii=True)
o3d.io.write_point_cloud(path + "half_cylinder_noisy_inverted_transformed.ply", half_cylinder_pcd_inverted,write_ascii=True)
o3d.io.write_point_cloud(path + "half_sphere_noisy_inverted_transformed.ply", half_sphere_pcd_inverted,write_ascii=True)
o3d.io.write_point_cloud(path + "combined_square_inverted_noisy_transformed.ply", combined_pcd_inverted,write_ascii=True)


# Visualize point clouds
# o3d.visualization.draw_geometries([square_pcd], window_name="Square (Noisy and Transformed)")
# o3d.visualization.draw_geometries([half_cylinder_pcd], window_name="Half-Cylinder Surface (Noisy and Transformed)")
# o3d.visualization.draw_geometries([half_sphere_pcd], window_name="Half-Sphere Surface (Noisy and Transformed)")
# o3d.visualization.draw_geometries([combined_pcd], window_name="combined-square Surface (Noisy and Transformed)")
