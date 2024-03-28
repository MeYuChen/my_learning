import numpy as np
import matplotlib.pyplot as plt

def get_corners(box):
    x = box[0]
    y = box[1]
    w = box[2]
    l = box[3]
    yaw = box[4]
    if yaw < 0:  # 映射到[0, pi)
        yaw += np.pi

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

def visualize_box(box):
    corners = get_corners(box)
    x_center, y_center = box[0], box[1]
    angle = box[4] * 180 / np.pi  # 将弧度转换为角度

    plt.figure()
    plt.plot(corners[:, 0], corners[:, 1], 'ro')  # 绘制四个角点
    plt.plot([corners[0, 0], corners[-1, 0]], [corners[0, 1], corners[-1, 1]], 'r-')  # 绘制框的边界
    for i in range(len(corners) - 1):
        plt.plot([corners[i, 0], corners[i+1, 0]], [corners[i, 1], corners[i+1, 1]], 'r-')

    # 绘制中心点
    plt.plot(x_center, y_center, 'bx')
    plt.text(x_center, y_center + 0.1, f'Center({x_center:.2f}, {y_center:.2f})', color='blue')

    # 绘制角度
    plt.text(x_center, y_center - 0.1, f'Angle: {angle:.2f}°', color='green')

    # 标记每个顶点坐标
    for i, corner in enumerate(corners):
        plt.text(corner[0], corner[1], f'({corner[0]:.2f}, {corner[1]:.2f})', color='blue')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Box Visualization')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# 测试用例

box = [0, 0, 2, 1, -np.pi/2]  # x, y, width, length, yaw
visualize_box(box)
