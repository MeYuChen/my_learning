import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def transform_obstacle_to_car_frame(car_x, car_y, car_yaw, obstacle_x, obstacle_y):
    # 计算障碍物相对于车体坐标系的位置
    obstacle_x_car = (obstacle_x - car_x) * np.cos(car_yaw) - (obstacle_y - car_y) * np.sin(car_yaw)
    obstacle_y_car = (obstacle_x - car_x) * np.sin(car_yaw) + (obstacle_y - car_y) * np.cos(car_yaw)

    return obstacle_x_car, obstacle_y_car

class PolygonVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Coordinate Transformation Visualization')
        self.ax.grid(True)

    def visualize_polygon(self, vertices, label=None, color='blue'):
        polygon = Polygon(vertices, closed=True, edgecolor='black', facecolor=color, label=label)
        self.ax.add_patch(polygon)

    def visualize(self):
        self.ax.legend()
        plt.show()

# 测试
if __name__ == "__main__":
    # 创建可视化器对象
    visualizer = PolygonVisualizer()

    # 定义车辆的世界坐标和航向角
    car_x, car_y = 2, 2
    car_yaw = np.deg2rad(90)

    # 定义障碍物的世界坐标
    obstacle_x, obstacle_y = 5, 5

    # 将障碍物坐标转换为车体坐标系下的坐标
    obstacle_x_car, obstacle_y_car = transform_obstacle_to_car_frame(car_x, car_y, car_yaw, obstacle_x, obstacle_y)
    print("障碍物在车体坐标系下的坐标：", obstacle_x_car, obstacle_y_car)

    # 可视化车辆和障碍物
    car_vertices = np.array([[car_x + 2, car_y + 1], [car_x + 2, car_y - 1], [car_x - 2, car_y - 1], [car_x - 2, car_y + 1]])
    obstacle_vertices_world = np.array([[obstacle_x + 1.5, obstacle_y + 0.5], [obstacle_x + 1.5, obstacle_y - 0.5], [obstacle_x - 1.5, obstacle_y - 0.5], [obstacle_x - 1.5, obstacle_y + 0.5]])
    obstacle_vertices_car = np.array([[obstacle_x_car + 1.5, obstacle_y_car + 0.5], [obstacle_x_car + 1.5, obstacle_y_car - 0.5], [obstacle_x_car - 1.5, obstacle_y_car - 0.5], [obstacle_x_car - 1.5, obstacle_y_car + 0.5]])

    visualizer.visualize_polygon(car_vertices, label='Car (World Frame)', color='blue')
    visualizer.visualize_polygon(obstacle_vertices_world, label='Obstacle (World Frame)', color='red')
    visualizer.visualize_polygon(obstacle_vertices_car, label='Obstacle (Car Frame)', color='green')

    # 显示图像
    visualizer.visualize()
