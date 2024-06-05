import pyvista as pv

# Path to your .ply file
file_path = "../data/pointclouds/transformed_pointcloud2.ply"

# Load the .ply file
mesh = pv.read(file_path)

# Create a plotter
plotter = pv.Plotter()

# Add the mesh to the plotter
plotter.add_mesh(mesh)

# Show the plotter
plotter.show()
