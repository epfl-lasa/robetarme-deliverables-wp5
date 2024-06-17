import numpy as np
import matplotlib.pyplot as plt

# Parameters
A = 0.15
mu = 10
sigma = 5 / np.sqrt(2)  # Solved algebraically

# Define the Gaussian function with the solved parameters
def gaussian_function(x, A, mu, sigma):
    return A * np.exp(-((x - mu) ** 2) / (2 * sigma ** 2))

# Generate x values
x = np.linspace(0, 20, 1000)

# Compute the Gaussian function values
y =A - gaussian_function(x, A, mu, sigma)

# Plot the function
plt.figure(figsize=(10, 6))
plt.plot(x, y, label=f'$f(x) = {A:.2f} \\exp\\left(-\\frac{{(x - 10)^2}}{{2 \\times {sigma:.2f}^2}}\\right)$')
plt.axvline(x=5, color='r', linestyle='--', label='$x=5$')
plt.axvline(x=10, color='g', linestyle='--', label='$x=10$')
plt.axvline(x=15, color='b', linestyle='--', label='$x=15$')
plt.axhline(y=0.15, color='gray', linestyle='--', label='$y=\\frac{1}{0.15}$')
plt.xlabel('x')
plt.ylabel('f(x)')
plt.title('Gaussian-shaped Function')
plt.legend()
plt.grid(True)
plt.show()
