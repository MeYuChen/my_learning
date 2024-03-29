
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 定义绕z轴旋转的旋转矩阵
def rotation_matrix_z(theta):
    return np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# 创建矩形的函数，基于中心点、宽度、高度和旋转角度
def create_rectangle(center, width, height, angle):
    return patches.Rectangle(
        (center[0] - width / 2, center[1] - height / 2), width, height,
        angle=angle, linewidth=1, edgecolor='r', facecolor='none'
    )

# 定义障碍物和车辆的尺寸和方向
obstacle_dimensions = (1, 0.5)
vehicle_dimensions = (1.5, 1)
obstacle_yaw = np.pi / 2  # 90度
vehicle_yaw = np.pi / 4*0  # 45度

# 定义障碍物点和旋转角度
obstacle_point = np.array([1, 1, 0])
vehicle_point = np.array([4, 4, 0])
rotation_angle = vehicle_yaw

# 计算旋转矩阵的转置（因为是坐标系在旋转）
rot_matrix_transpose = rotation_matrix_z(rotation_angle)

# 应用旋转到障碍物点（实际上是旋转坐标系）
obspoint_in_local = obstacle_point - vehicle_point
rotated_obstacle_point = rot_matrix_transpose.dot(obspoint_in_local)

# 绘制原始和旋转后的坐标系以及障碍物和车辆的形状
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# 原始位置和形状
obstacle_rect = create_rectangle(obstacle_point[:2], *obstacle_dimensions, np.degrees(obstacle_yaw))
vehicle_rect = create_rectangle(vehicle_point[:2], *vehicle_dimensions, np.degrees(vehicle_yaw))
ax1.add_patch(obstacle_rect)
ax1.add_patch(vehicle_rect)
ax1.set_xlim(-3, 3)
ax1.set_ylim(-3, 3)
ax1.set_title('原始位置和形状')
ax1.set_aspect('equal')
ax1.grid(True)  # 在第一个子图中加入网格

# 变换后的位置和形状
vehicle_rect = create_rectangle((0, 0), *vehicle_dimensions, np.degrees(vehicle_yaw))
ax2.add_patch(vehicle_rect)
rotated_obstacle_rect = create_rectangle(rotated_obstacle_point[:2], *obstacle_dimensions, np.degrees(obstacle_yaw))
ax2.add_patch(rotated_obstacle_rect)
ax2.set_xlim(-3, 3)
ax2.set_ylim(-3, 3)
ax2.set_title('变换后的位置和形状')
ax2.set_aspect('equal')
ax2.grid(True)  # 在第二个子图中加入网格

plt.show()
