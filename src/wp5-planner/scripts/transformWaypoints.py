# %matplotlib widget
import numpy as np
import matplotlib.pyplot as plt
import rospkg
from scipy.interpolate import interp1d
from scipy.optimize import minimize
import iteration_mapping as iter

def compute_quaternion(normal_vector):
    """
    Compute the quaternion for rotating the z-axis to align with the given normal vector.

    Parameters:
    normal_vector (numpy.ndarray): The normal vector to which the z-axis should be aligned.

    Returns:
    numpy.ndarray: The quaternion in the format [q_w, q_x, q_y, q_z].
    """
    # Normalize the input vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    # Define the z-axis of the original coordinate system
    nz = np.array([0, 0, 1])

    # Compute the rotation axis (cross product of z_axis and normal_vector)
    rotation_axis = np.cross(nz, normal_vector)

    # Compute the angle of rotation using the dot product
    cos_theta = np.dot(nz, normal_vector)  # Since both vectors are normalized
    theta = np.arccos(cos_theta)

    # Normalize the rotation axis
    rotation_axis_normalized = rotation_axis / np.linalg.norm(rotation_axis)

    # Compute the quaternion components
    q_w = np.cos(theta / 2)
    q_xyz = np.sin(theta / 2) * rotation_axis_normalized

    # Quaternion components
    QuaternionWxyz = np.array([q_w, q_xyz[0], q_xyz[1], q_xyz[2]])

    return QuaternionWxyz


def main():
     # Initialize the ROS package manager
    rospack = rospkg.RosPack()

    package_path = rospack.get_path('wp5_planner')
    boundary_path = package_path + '/data/boundary/'
    UV_path = package_path + '/data/UVmap/'
    point_path = package_path + '/data/paths/'
    save_data_path = point_path
    load_name='uv_map_pointcloud_target_transformed'

    # Load data
    txt_name = boundary_path + "planeData_"+ load_name + ".txt"
    # Load waypoints from the file
    planeData = np.loadtxt(txt_name)

    txt_name = boundary_path + "curveData_"+ load_name+".txt"
    # Load waypoints from the file
    curveData = np.loadtxt(txt_name)

    txt_name = UV_path + "normvector_pointcloud_target_transformed.txt"
    curveData_ori_vector = np.loadtxt(txt_name)

    distance_between_surf = planeData[:,0].mean() 


    txt_name = boundary_path + "boundary_planeData_"+ load_name+".txt"
    # Load waypoints from the file
    boundary = np.loadtxt(txt_name)
    # Create a new array filled with 0.1 with the same number of rows as boundary
    new_column = np.full((boundary.shape[0], 1), distance_between_surf)
    # Add the new column to boundary
    boundary = np.hstack((boundary, new_column))
    # change the waypoints to z x y
    boundary_3D = np.roll(boundary, 1, axis=1)

    txt_name = point_path + "waypointInFeatureSpace.txt"
    # Load waypoints from the file
    waypoints = np.loadtxt(txt_name)
    #change all z in waypoints to distance_between_surf
    waypoints[:, 2] = distance_between_surf
    # change the waypoints to z x y
    waypoints_3D = np.roll(waypoints, 1, axis=1)

    fig_width = 10  # inches
    fig_height = 4  # inches

    #  Interpolate new waypoints between the existing ones
    original_waypoints = waypoints_3D.T  # Transpose for easier manipulation
    new_waypoints = [original_waypoints[:, 0]]  # Initialize with the first waypoint

    for i in range(1, original_waypoints.shape[1]):
        distance = np.linalg.norm(original_waypoints[:, i] - original_waypoints[:, i-1])
        desired_interval = 0.18
        num_new_points = max(2, int(np.ceil(distance / desired_interval)))

        # Linear interpolation to create more waypoints
        interp_x = np.linspace(original_waypoints[0, i-1], original_waypoints[0, i], num=num_new_points)
        interp_y = np.linspace(original_waypoints[1, i-1], original_waypoints[1, i], num=num_new_points)
        interp_z = np.linspace(original_waypoints[2, i-1], original_waypoints[2, i], num=num_new_points)

        # Exclude the first point to avoid duplication
        new_waypoints.extend(np.vstack([interp_x[1:], interp_y[1:], interp_z[1:]]).T)

    # Convert list of arrays to a single NumPy array
    new_waypoints = np.array(new_waypoints)

    para = np.array([100, 0.6, 0.95])  # iteration algorithm parameter: [K, \mu, \beta]

    orinetation_plane = np.array([0.0000, -0.7071, 0.0000, 0.7071] * curveData.shape[0]).reshape(curveData.shape[0], 4)
    planeData = np.concatenate((planeData, orinetation_plane), axis=1)  # Concatenate curveData and orinetation along columns

    #--- calculate the quenternion for the curveData based on the ori vector
    orinetation = np.zeros((curveData_ori_vector.shape[0], 4))
    for i in range(curveData_ori_vector.shape[0]):
        orinetation[i,:] = compute_quaternion(curveData_ori_vector[i, :])


    curveData = np.concatenate((curveData, orinetation), axis=1)  # Concatenate curveData and orinetation along columns


    print('curveData shape:', curveData.shape)
    print('planeData shape:', planeData.shape)

    M = iter.iteration(para, planeData, curveData) # Training

    orinetation_plane = np.array([0.0000, -0.7071, 0.0000, 0.7071] * new_waypoints.shape[0]).reshape(new_waypoints.shape[0], 4)
    new_waypoints = np.concatenate((new_waypoints, orinetation_plane), axis=1)  # Concatenate curveData and orinetation along columns

    print('new_waypoints:', new_waypoints.shape)
    waypoints_mapped = M.forward(new_waypoints) 

    filename_curve = f'{save_data_path}waypointInOriSpace.txt'
    np.savetxt(filename_curve, waypoints_mapped[:,:], delimiter='\t')  # Transpose to save in correct format
    print('Saved new_waypoints in original space to', filename_curve)
