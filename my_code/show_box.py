import numpy as np
import matplotlib.pyplot as plt

def get_corners(box):
    x = box[0]
    y = box[1]
    w = box[2]
    l = box[3]
    yaw = box[4]
    if yaw < 0:
        yaw = yaw + np.pi

    bev_corners = np.zeros((4, 2), dtype=np.float32)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    bev_corners[0, 0] = (w / 2) * cos_yaw - (l / 2) * sin_yaw + x
    bev_corners[0, 1] = (w / 2) * sin_yaw + (l / 2) * cos_yaw + y

    bev_corners[1, 0] = (l / 2) * sin_yaw + (w / 2) * cos_yaw + x
    bev_corners[1, 1] = (w / 2) * sin_yaw - (l / 2) * cos_yaw + y

    bev_corners[2, 0] = (-w / 2) * cos_yaw - (-l / 2) * sin_yaw + x
    bev_corners[2, 1] = (-w / 2) * sin_yaw + (-l / 2) * cos_yaw + y

    bev_corners[3, 0] = (-l / 2) * sin_yaw + (-w / 2) * cos_yaw + x
    bev_corners[3, 1] = (-w / 2) * sin_yaw - (-l / 2) * cos_yaw + y

    return bev_corners
import numpy as np

def get_corners_optimized(box):
    x, y, w, l, yaw = box
    # if yaw < 0:
    #     yaw += np.pi

    # 创建旋转矩阵，确保旋转方向正确
    rotation_matrix = np.array([
        [np.cos(yaw), np.sin(yaw)],
        [-np.sin(yaw), np.cos(yaw)]
    ])

    # 定义相对于中心的角点
    corner_offsets = np.array([
        [w / 2, l / 2],
        [-w / 2, l / 2],
        [-w / 2, -l / 2],
        [w / 2, -l / 2]
    ])

    # 旋转并平移角点
    corners = np.dot(corner_offsets, rotation_matrix) + np.array([x, y])

    return corners


# 测试优化后的函数
test_box = [4, 4, 4, 2, np.pi/-3]
corners = get_corners_optimized(test_box)
# print("优化后的角点:\n", corners)


# 定义测试边界框
test_box = [2, 2, 4, 2, np.pi/2]  # 示例：x, y, 宽度, 长度, yaw

# 使用get_corners函数计算角点
# corners = get_corners(test_box)

# 可视化角点
plt.figure()
plt.plot(corners[:, 0], corners[:, 1], 'ro')  # 角点
# 绘制边界框的边
for i in range(4):
    plt.plot([corners[i, 0], corners[(i + 1) % 4, 0]], [corners[i, 1], corners[(i + 1) % 4, 1]], 'r-')

# 标出顶点坐标
for i, corner in enumerate(corners):
    plt.text(corner[0], corner[1], f'({corner[0]:.2f}, {corner[1]:.2f})', color='blue')

# 计算并标出中心坐标
center_x = np.mean(corners[:, 0])
center_y = np.mean(corners[:, 1])
plt.plot(center_x, center_y, 'bo')  # 中心点
plt.text(center_x, center_y, f'({center_x:.2f}, {center_y:.2f}, {test_box[4]:.2f} rad)', color='green')

# 设置图形属性
plt.xlim(0, 5)
plt.ylim(0, 5)
plt.gca().set_aspect('equal', adjustable='box')
plt.title('测试边界框的角点及中心')
plt.show()

#现在