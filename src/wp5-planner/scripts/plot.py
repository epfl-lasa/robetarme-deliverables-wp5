import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to read the .txt file and return the coordinates
def read_coordinates(file_path):
    data = np.loadtxt(file_path)  # For space-separated values, no need to specify delimiter
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]
    return x, y, z

# Function to plot the coordinates
def plot_coordinates(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

# Main function
def main():
    file_path = '../data/boundary/boundary_planeData_uv_map_pointcloud_target_transformed.txt'  # Replace with your file path
    x, y, z = read_coordinates(file_path)
    plot_coordinates(x, y, z)

if __name__ == '__main__':
    main()

