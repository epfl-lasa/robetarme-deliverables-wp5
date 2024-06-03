# ExtractPolygons
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
from scipy.interpolate import interp1d
import math
import rospkg



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



def main():

    # Initialize the ROS package manager
    rospack = rospkg.RosPack()

    package_path = rospack.get_path('wp5_planner')
    file_name = 'uv_map_pointcloud_target_transformed'

    load_data_path = package_path + '/data/UVmap/'
    save_data_path = package_path + '/data/boundary/'

    # Load data from a text file
    txt_name = load_data_path + file_name + '.txt'
    Omega = np.loadtxt(txt_name).T  # Transpose to match MATLAB format

    cuverData = Omega[3:6, :]

    distance_between_surf=0.3

    planeData_raw = np.vstack((np.full(Omega.shape[1], distance_between_surf), Omega[1], Omega[2]))

    # Perform rotation and transformations as in the MATLAB code
    rotM = euler_angle_to_rotation_matrix([0,0,(-90/360)*(2*math.pi)])
    planeData_rote = np.dot(rotM, planeData_raw)

    center_cuverData = np.mean(cuverData, axis=1)
    center_planeData_raw = np.mean(planeData_rote, axis=1)
    centroid_difference = center_cuverData - center_planeData_raw + np.array([distance_between_surf, 0, 0])
    centroid_difference_rep = np.tile(centroid_difference[:, None], planeData_rote.shape[1])
    planeData = planeData_rote + centroid_difference_rep

    # Boundary and enhanced visualization
    Y = planeData[1, :]
    Z = planeData[2, :]
    hull = ConvexHull(np.vstack((Y, Z)).T)
    boundary_planeData = np.vstack((Y[hull.vertices], Z[hull.vertices], np.full(len(hull.vertices), 0.1)))

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
