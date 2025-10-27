import numpy as np
from scipy.interpolate import PchipInterpolator
import matplotlib.pyplot as plt


WAYPOINTS = [
    [0.0, 0.0, 0.5, 0.0],
    [0, 0.0, 1.5, 0],
    [-0.25, 0.0, 1.5, 0],
    [-1.0, 0.0, 1.5, 0]
]
dt = 25

# Sample 3D data points
x = np.array([point[0] for point in WAYPOINTS])
y = np.array([point[1] for point in WAYPOINTS]) 
z = np.array([point[2] for point in WAYPOINTS])

# Create PCHIP interpolators for each dimension
pchip_x = PchipInterpolator(np.arange(len(x)), x)
pchip_y = PchipInterpolator(np.arange(len(y)), y)
pchip_z = PchipInterpolator(np.arange(len(z)), z)

# Define parameter values for interpolation (t represents the curve parameter)
t_original = np.arange(len(x))
t_new = np.linspace(0, len(x)-1, dt)

# Interpolate all coordinates
x_new = pchip_x(t_new)
y_new = pchip_y(t_new)
z_new = pchip_z(t_new)

# array: {[x, y, z, yaw], ...}
trajectory = np.array([x_new, y_new, z_new]).T
print(trajectory)

# Calculate derivatives for each dimension
x_derivative = pchip_x.derivative()(t_new)
y_derivative = pchip_y.derivative()(t_new)
z_derivative = pchip_z.derivative()(t_new)

# Create 3D plot
fig = plt.figure(figsize=(12, 8))

# 3D trajectory plot
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.plot(x, y, z, 'o', label='Original data', markersize=8)
ax1.plot(x_new, y_new, z_new, label='PCHIP Interpolation')
ax1.set_title('3D Trajectory')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.legend()

# Individual coordinate plots
plt.subplot(2, 2, 2)
plt.plot(t_new, x_new, label='X interpolation')
plt.plot(t_new, y_new, label='Y interpolation')
plt.plot(t_new, z_new, label='Z interpolation')
plt.plot(t_original, x, 'ro', markersize=6)
plt.plot(t_original, y, 'go', markersize=6)
plt.plot(t_original, z, 'bo', markersize=6)
plt.title('Coordinates vs Parameter')
plt.xlabel('Parameter t')
plt.ylabel('Coordinate Value')
plt.legend()

# Derivative plots
plt.subplot(2, 2, 3)
plt.plot(t_new, x_derivative, 'r', label="X' derivative")
plt.plot(t_new, y_derivative, 'g', label="Y' derivative")
plt.plot(t_new, z_derivative, 'b', label="Z' derivative")
plt.title('Derivatives')
plt.xlabel('Parameter t')
plt.ylabel("Derivative Value")
plt.legend()

# Magnitude of velocity (speed)
speed = np.sqrt(x_derivative**2 + y_derivative**2 + z_derivative**2)
plt.subplot(2, 2, 4)
plt.plot(t_new, speed, 'purple', label='Speed')
plt.title('Speed along trajectory')
plt.xlabel('Parameter t')
plt.ylabel('Speed')
plt.legend()

plt.tight_layout()
plt.show()