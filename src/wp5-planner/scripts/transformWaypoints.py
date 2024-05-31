# %matplotlib widget
import numpy as np
import matplotlib.pyplot as plt
import rospkg
from scipy.interpolate import interp1d


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
    boundary_path = package_path + '/data/boudary/'
    # load_name='flat'
    load_name='uv_map_pointcloud_target_transformed'


    Run2D_Test1_Real3D2 = 2

    # Assuming 'save_data_path' is defined somewhere in your code
    txt_name = boundary_path + "boundary_planeData_"+ load_name + ".txt"
    # Load waypoints from the file
    planeData = np.loadtxt(txt_name)


    distance_between_surf = planeData[:,0].mean() 





    orinetation_plane = np.array([0.0000, -0.7071, 0.0000, 0.7071] * new_waypoints.shape[0]).reshape(new_waypoints.shape[0], 4)
    new_waypoints = np.concatenate((new_waypoints, orinetation_plane), axis=1)  # Concatenate curveData and orinetation along columns

    print('new_waypoints:', new_waypoints.shape)
    waypoints_mapped = M.forward(new_waypoints) 

    filename_curve = f'{save_data_path}waypointInOriSpace.txt'
    np.savetxt(filename_curve, waypoints_mapped[:,:], delimiter='\t')  # Transpose to save in correct format
    print('Saved new_waypoints in original space to', filename_curve)
