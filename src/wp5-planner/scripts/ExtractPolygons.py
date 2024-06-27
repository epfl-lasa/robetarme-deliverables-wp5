# ExtractPolygons
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
from scipy.interpolate import interp1d
import math
import rospkg
from scipy.optimize import minimize_scalar

def calculate_total_distance_3D(points):
    """
    Calculate the total distance traveled through a set of 3D points in the given order.
    
    Args:
    points (list of tuples): A list where each tuple represents the x, y, and z coordinates of a point.
    
    Returns:
    float: The total distance traveled from the first to the last point, visiting each sequentially.
    """
    total_distance = 0
    # Loop through the list of points and calculate distances between consecutive points
    for i in range(len(points) - 1):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[i + 1]
        
        # Calculate Euclidean distance between the current point and the next
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        total_distance += distance
    
    return total_distance
    
def total_distance_squared(theta, points1, points2):
    total_distance = 0
    for (x1, y1, z1), (x2, y2, z2) in zip(points1, points2):
        dy = y1 * np.cos(theta) - z1 * np.sin(theta) - y2
        dz = y1 * np.sin(theta) + z1 * np.cos(theta) - z2
        total_distance += (x1 - x2) ** 2 + dy ** 2 + dz ** 2
    return total_distance

def find_optimal_rotation(points1, points2):
    result = minimize_scalar(total_distance_squared, args=(points1, points2), bounds=(0, 2*np.pi), method='bounded')
    optimal_distance = result.fun  # optimized total distance value
    return np.degrees(result.x), optimal_distance

def euler_angle_to_rotation_matrix(theta_ori_zyx):
    """
    Convert Euler angles (ZYX order) to a rotation matrix.
    
    Parameters:
        theta_ori_zyx (array-like): A list or array of Euler angles [e_z, e_y, e_x].
    
    Returns:
        numpy.ndarray: The rotation matrix corresponding to the Euler angles.
    """
    e_z, e_y, e_x = theta_ori_zyx  # Unpack the Euler angles
    
    c = np.cos([e_z, e_y, e_x])
    s = np.sin([e_z, e_y, e_x])
    
    # Define the rotation matrix components
    RotMatrix = np.array([
        [c[1]*c[0], s[2]*s[1]*c[0] - c[2]*s[0], c[2]*s[1]*c[0] + s[2]*s[0]],
        [c[1]*s[0], s[2]*s[1]*s[0] + c[2]*c[0], c[2]*s[1]*s[0] - s[2]*c[0]],
        [-s[1],      s[2]*c[1],                c[2]*c[1]]
    ])
    
    return RotMatrix

def process_data_for_mapping(Omega, distance_between_surf):
    #--- Omega is the data [1:2,:] is plane data, [3:6,:] is cuver data
    
    cuverData = Omega[3:6, :]

    planeData_raw = np.vstack((np.full(Omega.shape[1], distance_between_surf), Omega[1], Omega[2]))

    #--- find the center of surface and replace the planeData_raw
    center_cuverData = np.mean(cuverData, axis=1)
    center_planeData_raw = np.mean(planeData_raw, axis=1)
    centroid_difference = center_cuverData - center_planeData_raw + np.array([distance_between_surf, 0, 0])
    centroid_difference_rep = np.tile(centroid_difference[:, None], planeData_raw.shape[1])

    #--- try find the best rotate for the planeData_raw and cuverData
    planeData_centered = planeData_raw - center_planeData_raw.reshape(-1, 1)
    cuverData_centered = cuverData - center_cuverData.reshape(-1, 1)
    selected_planePoints = np.concatenate((planeData_centered[:, :10], planeData_centered[:, -10:]), axis=1)
    selected_cuverPoints = np.concatenate((cuverData_centered[:, :10], cuverData_centered[:, -10:]), axis=1)

    theta_optimal, dist_optimal = find_optimal_rotation(selected_planePoints.T, selected_cuverPoints.T)
    print(f"Optimal rotation angle: {theta_optimal} degrees")

    # Perform rotation and transformations as in the MATLAB code
    rotM = euler_angle_to_rotation_matrix([0,0,(theta_optimal/360)*(2*math.pi)])

    planeData_rote = np.dot(rotM, planeData_centered)
    

    #--- scale the planeData to cuverData
    #--- find the best scale for planeData
    total_dist_cuverData = calculate_total_distance_3D(cuverData.T)
    total_dist_planeData = calculate_total_distance_3D(planeData_rote.T)
    scale_factor = total_dist_cuverData / total_dist_planeData

    planeData_scaled = planeData_rote * scale_factor *1.3

    planeData = planeData_scaled + center_planeData_raw.reshape(-1, 1)
    planeData = planeData + centroid_difference_rep

    


    return cuverData,planeData, dist_optimal


def main():

    # Initialize the ROS package manager
    rospack = rospkg.RosPack()

    package_path = rospack.get_path('wp5_planner')
    file_name = 'uv_map_pointcloud_target_transformed'
    file_name_vector = 'normvector_pointcloud_target_transformed'


    load_data_path = package_path + '/data/UVmap/'
    save_data_path = package_path + '/data/boundary/'

    # Load data from a text file
    txt_name = load_data_path + file_name + '.txt'
    Omega = np.loadtxt(txt_name).T  # Transpose to match MATLAB format

    txt_name_vector = load_data_path + file_name_vector + '.txt'
    normals_vector = np.loadtxt(txt_name_vector)  # Transpose to match MATLAB format

    distance_between_surf=0.3

    #--- process data for mapping, rotate, move to same center
    use_new_scaleFunction = True
    if use_new_scaleFunction:

        # Create copies of Omega for Omega1 and Omega2
        Omega1 = Omega.copy()
        Omega2 = Omega.copy()

        Omega2[1] = Omega[2]  # Overwrite row 1 with row 2
        Omega2[2] = Omega[1]  # Overwrite row 2 with the stored row 1

        cuverData1, planeData1, dist_optimal1 = process_data_for_mapping(Omega1, distance_between_surf)
        cuverData2, planeData2, dist_optimal2 = process_data_for_mapping(Omega2, distance_between_surf)

        if dist_optimal2 >dist_optimal1:
            cuverData = cuverData1
            planeData = planeData1
        else:
            cuverData = cuverData2
            planeData = planeData2
            normals_vector = -1*normals_vector
            np.savetxt(load_data_path +file_name_vector+ ".txt", normals_vector, fmt='%f', delimiter=' ')

    else:
        cuverData = Omega[3:6, :]
        
        planeData_raw = np.vstack((np.full(Omega.shape[1], distance_between_surf), Omega[1], Omega[2]))
    
        # Perform rotation and transformations as in the MATLAB code
        rotM = euler_angle_to_rotation_matrix([0,0,(-90/360)*(2*math.pi)])
        planeData_rote = np.dot(rotM, planeData_raw)
    
        center_cuverData = np.mean(cuverData, axis=1)
        center_planeData_raw = np.mean(planeData_rote, axis=1)
        centroid_difference = center_cuverData - center_planeData_raw + np.array([distance_between_surf, 0, 0])
        centroid_difference_rep = np.tile(centroid_difference[:, None], planeData_rote.shape[1])
        planeData = planeData_rote + centroid_difference_rep

    # Creating a figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scatter1 = ax.scatter(cuverData[0, :], cuverData[1, :], cuverData[2, :], c='b',s=1)
    scatter2 = ax.scatter(planeData[0, :], planeData[1, :], planeData[2, :], c='g',s=1)
    # Choose a number of points to plot
    num_points = 100  # Change this to the number of points you want
    # Generate random indices
    indices = np.random.choice(cuverData.shape[1], size=num_points, replace=False)
    # Plot the selected points
    for i in indices:
        ax.plot([cuverData[0, i], planeData[0, i]], [cuverData[1, i], planeData[1, i]], [cuverData[2, i], planeData[2, i]], 'r-')
    # Adding labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.axis('equal')
    # Show plot
    plt.show()


    # Boundary and enhanced visualization
    Y = planeData[1, :]
    Z = planeData[2, :]
    hull = ConvexHull(np.vstack((Y, Z)).T)

    boundary_planeData = np.vstack((Y[hull.vertices], Z[hull.vertices], np.full(len(hull.vertices), 0.1)))


    plt.figure()
    plt.scatter(Y, Z, c='g', alpha=0.5,s=1)
    plt.plot(Y[hull.vertices], Z[hull.vertices], 'r-')
    plt.xlabel('Y-axis')
    plt.ylabel('Z-axis')
    plt.axis('equal')
    plt.show()


    filename = save_data_path+"planeData_"+ file_name+".txt"

    np.savetxt(filename, boundary_planeData.T, delimiter='\t')  # Transpose to save in correct format
    
    filename = save_data_path+"boundary_planeData_"+ file_name+".txt"
    np.savetxt(filename, boundary_planeData.T, delimiter='\t')  # Transpose to save in correct format
    filename_plant = save_data_path+"planeData_"+ file_name+".txt"
    np.savetxt(filename_plant, planeData.T, delimiter='\t')  # Transpose to save in correct format
    filename_curve = save_data_path+"curveData_"+ file_name+".txt"
    np.savetxt(filename_curve, cuverData.T, delimiter='\t')  # Transpose to save in correct format

if __name__ == '__main__':
    main()
