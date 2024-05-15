# # import matplotlib.pyplot as plt
# # from matplotlib.patches import Rectangle, Circle
# # import numpy as np
# # import math

# # def vector_projection(point, vector):
# #     # 计算点 P 到向量 v 的投影
# #     dot_product = np.dot(point, vector)  # 点积
# #     vector_length_squared = np.dot(vector, vector)  # 向量长度的平方
# #     projection_length = dot_product / vector_length_squared  # 投影长度
# #     projection_vector = projection_length * vector  # 投影向量
# #     proj_len = math.sqrt(projection_vector[0]*projection_vector[0]*projection_vector[1]*projection_vector[1] )
# #     v1 = np.array(projection_vector)
# #     v2 = np.array(vector)
# #     if np.dot(v1,v2)<0:
# #         proj_len = -proj_len
# #     return proj_len,projection_vector

# # def is_overlap(rect, circle_center, radius):
# #     # 计算矩形的中心点
# #     rect_center = [(rect[0][0] + rect[1][0]) / 2, (rect[0][1] + rect[1][1]) / 2]
# #     x1 = rect[0][0]
# #     x2 = rect[1][0]
# #     y1 = rect[0][1]
# #     y2 = rect[1][1]
# #     edge = [(x2 - x1,0),(0,y1 - y2),(x1 - x2,0),(0,y2 - y1)]

    
# #     # 计算圆心到矩形中心的距离的平方
# #     distance_squared = (circle_center[0] - rect_center[0]) ** 2 + (circle_center[1] - rect_center[1]) ** 2

# #     # 计算圆心到矩形的最近点的距离的平方
# #     closest_x = max(min(circle_center[0], rect[0][0]), min(circle_center[0], rect[1][0]))
# #     closest_y = max(min(circle_center[1], rect[1][1]), min(circle_center[1], rect[0][1]))
# #     print(closest_x,closest_y)
# #     distance_squared_closest = (circle_center[0] - closest_x) ** 2 + (circle_center[1] - closest_y) ** 2

# #     # 如果最近点的距离小于等于半径，则相交
# #     return distance_squared_closest <= radius ** 2 or distance_squared <= radius ** 2

# # # 定义矩形和圆的参数
# # rect = [(0, 0), (2, 2)]  # 矩形左上角和右下角坐标
# # circle_center = (-1, 1)  # 圆心坐标
# # radius =1 # 圆的半径

# # # 判断是否相交
# # overlap = is_overlap(rect, circle_center, radius)
# # print("圆和矩形相交：" if overlap else "圆和矩形不相交")

# # # 可视化
# # fig, ax = plt.subplots()

# # # 绘制矩形
# # rect_patch = Rectangle(rect[0], rect[1][0] - rect[0][0], rect[1][1] - rect[0][1], edgecolor='b', facecolor='none')
# # ax.add_patch(rect_patch)

# # # 绘制圆
# # circle_patch = Circle(circle_center, radius, edgecolor='r', facecolor='none')
# # ax.add_patch(circle_patch)

# # # 设置坐标轴范围和标签
# # ax.set_xlim(-1, 5)
# # ax.set_ylim(-1, 5)
# # ax.set_aspect('equal')
# # ax.set_xlabel('X')
# # ax.set_ylabel('Y')
# # ax.set_title('Overlap Detection')

# # # 显示图形
# # plt.grid(True)
# # plt.legend([rect_patch, circle_patch], ['Rectangle', 'Circle'])
# # plt.show()


        
# # # import numpy as np
# # # import matplotlib.pyplot as plt
# # # import math

# # # def vector_projection(point, vector):
# # #     # 计算点 P 到向量 v 的投影
# # #     dot_product = np.dot(point, vector)  # 点积
# # #     vector_length_squared = np.dot(vector, vector)  # 向量长度的平方
# # #     projection_length = dot_product / vector_length_squared  # 投影长度
# # #     projection_vector = projection_length * vector  # 投影向量
# # #     proj_len = math.sqrt(projection_vector[0]*projection_vector[0]*projection_vector[1]*projection_vector[1] )
# # #     v1 = np.array(projection_vector)
# # #     v2 = np.array(vector)
# # #     if np.dot(v1,v2)<0:
# # #         proj_len = -proj_len


# # #     return proj_len,projection_vector

# # # # 定义点 P 和向量 v
# # # point = np.array([0, -1])  # 点 P 的坐标
# # # vector = np.array([1, 1])  # 向量 v 的坐标

# # # # 计算点到向量的投影
# # # proj_len,projection = vector_projection(point, vector)

# # # print(proj_len)

# # # # 绘制图形
# # # plt.figure(figsize=(8, 6))

# # # # 绘制原始向量
# # # plt.quiver(0, 0, vector[0], vector[1], angles='xy', scale_units='xy', scale=1, color='b', label='Original Vector')

# # # # 绘制点 P
# # # plt.scatter(point[0], point[1], color='r', label='Point P')

# # # # 绘制点到向量的投影向量
# # # plt.quiver(0, 0, projection[0], projection[1], angles='xy', scale_units='xy', scale=1, color='g', label='Projection Vector')

# # # # 设置图例、坐标轴和标题
# # # plt.legend()
# # # plt.axhline(y=0, color='k', linewidth=0.5)
# # # plt.axvline(x=0, color='k', linewidth=0.5)
# # # plt.xlabel('X')
# # # plt.ylabel('Y')
# # # plt.title('Vector Projection')

# # # # 设置坐标轴范围
# # # plt.xlim(-1, 5)
# # # plt.ylim(-1, 5)

# # # # 显示图形
# # # plt.grid()
# # # plt.gca().set_aspect('equal', adjustable='box')
# # # plt.show()
# import matplotlib.pyplot as plt
# import numpy as np
# import math
# def check_overlap(radius, x_center, y_center, x1, y1, x2, y2):
#     tx = (x1 + x2) / 2
#     ty = (y1 + y2) / 2
#     tdx = -tx
#     tdy = -ty
    
#     # 变换矩形的左下角和右上角
#     x11 = x1 + tdx
#     x21 = x2 + tdx
#     y11 = y1 + tdy
#     y21 = y2 + tdy
    
#     # 变换圆心
#     xc = abs(x_center + tdx)
#     yc = abs(y_center + tdy)
    
#     # 计算向量 (ux, uy)
#     ux = xc - x21
#     uy = yc - y21
    
#     # 将 ux 和 uy 小于 0 的置为 0
#     ux = max(0, ux)
#     uy = max(0, uy)
    
#     return ux * ux + uy * uy <= radius * radius

# # 测试函数
# radius = math.sqrt(1)
# x_center, y_center = -1, 3 - math.sqrt(2)
# x1, y1 = 0, 0
# x2, y2 = 2, 2

# is_overlapping = check_overlap(radius, x_center, y_center, x1, y1, x2, y2)
# print(f"Is overlapping: {is_overlapping}")

# # 可视化
# fig, ax = plt.subplots()

# # 绘制矩形
# rect = plt.Rectangle((x1, y1), x2 - x1, y2 - y1, fill=None, edgecolor='b', linewidth=2)
# ax.add_patch(rect)

# # 绘制圆
# circle = plt.Circle((x_center, y_center), radius, color='r', fill=False, linewidth=2)
# ax.add_patch(circle)

# # 设置坐标范围和标题
# ax.set_xlim(-3, 3)
# ax.set_ylim(-3, 3)
# ax.set_aspect('equal', 'box')
# plt.title(f"Circle and Rectangle Overlap: {is_overlapping}")

# plt.grid()
# plt.show()
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
import math

def project_polygon(vertices, axis):
    dots = np.dot(vertices, axis)
    return [np.min(dots), np.max(dots)]

def project_circle(circle_center, radius, axis):
    circle_center_proj = np.dot(circle_center, axis)
    return [circle_center_proj - radius, circle_center_proj + radius]

def separating_axis_theorem(vertices, circle_center, radius):
    axes = []
    num_vertices = len(vertices)
    
    # Calculate the axes perpendicular to the edges of the polygon
    for i in range(num_vertices):
        edge = vertices[i] - vertices[i - 1]
        normal = np.array([-edge[1], edge[0]])
        if np.linalg.norm(normal) > 0:
            axes.append(normal / np.linalg.norm(normal))
    
    # Add axes from the circle center to each vertex of the polygon
    for vertex in vertices:
        axis = vertex - circle_center
        if np.linalg.norm(axis) > 0:
            axes.append(axis / np.linalg.norm(axis))
    
    # Project both polygons onto each axis and check for overlap
    for axis in axes:
        polygon_proj = project_polygon(vertices, axis)
        circle_proj = project_circle(circle_center, radius, axis)
        
        if polygon_proj[1] < circle_proj[0] or circle_proj[1] < polygon_proj[0]:
            return False
    
    return True

# Define rectangle vertices (oriented rectangle)
angle = np.deg2rad(45)
cos_angle = np.cos(angle)
sin_angle = np.sin(angle)
rect_center = np.array([2, 2])
rect_size = np.array([2, 2])
half_size = rect_size / 2

vertices = np.array([
    [-half_size[0], -half_size[1]],
    [half_size[0], -half_size[1]],
    [half_size[0], half_size[1]],
    [-half_size[0], half_size[1]]
])

rotation_matrix = np.array([
    [cos_angle, -sin_angle],
    [sin_angle, cos_angle]
])

rotated_vertices = np.dot(vertices, rotation_matrix) + rect_center

# Define circle
circle_center = np.array([0, 1])
radius = math.sqrt(2)

# Check for overlap
is_overlapping = separating_axis_theorem(rotated_vertices, circle_center, radius)
print(f"Circle and oriented rectangle overlapping: {is_overlapping}")

# Visualization
fig, ax = plt.subplots()

# Draw rectangle
polygon = Polygon(rotated_vertices, closed=True, fill=None, edgecolor='b', linewidth=2)
ax.add_patch(polygon)

# Draw circle
circle = Circle(circle_center, radius, color='r', fill=False, linewidth=2)
ax.add_patch(circle)

# Set plot limits and aspect ratio
ax.set_xlim(-1, 5)
ax.set_ylim(-1, 5)
ax.set_aspect('equal', 'box')
plt.title(f"Circle and Oriented Rectangle Overlap: {is_overlapping}")

plt.grid()
plt.show()
