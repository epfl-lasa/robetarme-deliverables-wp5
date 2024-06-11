import matplotlib.pyplot as plt
import numpy as np

# Load the path data from the file
data = np.loadtxt('../data/paths/waypointInOriSpaceConverted.txt')


# Extract positions (assuming positions are the first three columns)
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# Plot the path
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='Path')
ax.scatter(x, y, z, c='r', marker='o')  # Add points to indicate path vertices

# Annotate each point with its index
for i in range(len(x)):
    ax.text(x[i], y[i], z[i], f'{i}', size=10, zorder=1, color='k')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Path Plot')
ax.legend()

plt.show()