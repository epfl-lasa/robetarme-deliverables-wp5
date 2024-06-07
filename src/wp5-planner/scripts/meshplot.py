import trimesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Function to load and visualize the .obj file
def visualize_obj(file_path):
    # Load the mesh from the .obj file
    mesh = trimesh.load(file_path)
    
    # Extract vertices and faces
    vertices = mesh.vertices
    faces = mesh.faces
    
    # Create a figure for plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the vertices
    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], c='r', marker='o')
    
    # Plot the faces
    for face in faces:
        v0, v1, v2 = vertices[face]
        tri = [v0, v1, v2]
        tri = Poly3DCollection([tri], facecolors='w', edgecolors='k', linewidths=0.1, alpha=0.5)
        ax.add_collection3d(tri)
    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Show plot
    plt.show()

# Main function
def main():
    file_path = '../data/meshes/pointcloud_target_transformed.obj'  # Replace with your .obj file path
    visualize_obj(file_path)

if __name__ == '__main__':
    main()
