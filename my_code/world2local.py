import numpy as np
import matplotlib.pyplot as plt

# Define the rotation matrix for rotating the coordinate system around the z-axis
def rotation_matrix_z(theta):
    return np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# Define the obstacle point and the rotation angle (90 degrees or pi/2 radians)
obstacle_point = np.array([1, 0, 0])
vehicle_point = np.array([2, 2, 0])  # Adding vehicle position
rotation_angle = np.pi / 2

# Calculate the transpose of the rotation matrix (since the coordinate system is rotating)
rot_matrix_transpose = rotation_matrix_z(rotation_angle).T

# Apply the rotation to the obstacle point (which effectively rotates the coordinate system)
rotated_obstacle_point = rot_matrix_transpose.dot(obstacle_point)

# Plot the original and rotated coordinate systems with the obstacle and vehicle points
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Original coordinate system, vehicle, and obstacle points
ax1.quiver(0, 0, 1, 0, angles='xy', scale_units='xy', scale=1, color='r', label='X-axis')
ax1.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='g', label='Y-axis')
ax1.scatter(obstacle_point[0], obstacle_point[1], color='b', label='Obstacle (Original)')
ax1.scatter(vehicle_point[0], vehicle_point[1], color='orange', label='Vehicle (Original)')
ax1.set_xlim(-2, 4)
ax1.set_ylim(-2, 4)
ax1.set_aspect('equal')
ax1.set_title('Original Coordinate System')
ax1.legend()
ax1.grid(True)

# Rotated coordinate system and obstacle point relative to vehicle
ax2.quiver(vehicle_point[0], vehicle_point[1], rot_matrix_transpose[0, 0], rot_matrix_transpose[1, 0], angles='xy', scale_units='xy', scale=1, color='r')
ax2.quiver(vehicle_point[0], vehicle_point[1], rot_matrix_transpose[0, 1], rot_matrix_transpose[1, 1], angles='xy', scale_units='xy', scale=1, color='g')
ax2.scatter(vehicle_point[0] + rotated_obstacle_point[0], vehicle_point[1] + rotated_obstacle_point[1], color='b', label='Obstacle (Rotated)')
ax2.scatter(vehicle_point[0], vehicle_point[1], color='orange', label='Vehicle (Origin)')
ax2.set_xlim(-2, 4)
ax2.set_ylim(-2, 4)
ax2.set_aspect('equal')
ax2.set_title('Rotated Coordinate System')
ax2.legend()
ax2.grid(True)

# Show the plots
plt.show()
