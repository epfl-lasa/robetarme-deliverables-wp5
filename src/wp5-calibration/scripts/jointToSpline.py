import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# Step 1: Read the joint configurations from a .txt file
file_path = '../txts/path_calibration_camera_joint.txt'
joint_configurations = np.loadtxt(file_path)

# Define the time steps corresponding to each configuration
t = np.linspace(0, 1, len(joint_configurations))

# Step 2: Create a linear interpolation for each joint
linear_interpolations = [interp1d(t, joint_configurations[:, i], kind='linear') for i in range(joint_configurations.shape[1])]

# Generate points along the interpolation for smooth trajectory
t_fine = np.linspace(0, 1, 100)
trajectory = np.array([interp(t_fine) for interp in linear_interpolations]).T

# Step 3: Save the smoothed trajectory to a .txt file
output_file_path = '../txts/smoothed_trajectory.txt'
np.savetxt(output_file_path, trajectory)

# Optional: Plot the result for visualization
plt.figure()
for i in range(trajectory.shape[1]):
    plt.plot(t_fine, trajectory[:, i], label=f'Joint {i+1}')
    plt.scatter(t, joint_configurations[:, i])
plt.legend()
plt.xlabel('Time')
plt.ylabel('Joint Angles')
plt.title('Smooth Joint Trajectories')
plt.show()
