import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

class PolygonVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Polygon Visualization')
        self.ax.grid(True)
        self.ax.legend()
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)

    def _calculate_rectangle_vertices(self, x, y, length, width, yaw):
        vertices = np.array([
            [x + length / 2 * np.cos(yaw) - width / 2 * np.sin(yaw),
             y + length / 2 * np.sin(yaw) + width / 2 * np.cos(yaw)],
            [x + length / 2 * np.cos(yaw) + width / 2 * np.sin(yaw),
             y + length / 2 * np.sin(yaw) - width / 2 * np.cos(yaw)],
            [x - length / 2 * np.cos(yaw) + width / 2 * np.sin(yaw),
             y - length / 2 * np.sin(yaw) - width / 2 * np.cos(yaw)],
            [x - length / 2 * np.cos(yaw) - width / 2 * np.sin(yaw),
             y - length / 2 * np.sin(yaw) + width / 2 * np.cos(yaw)]
        ])
        return vertices

    def visualize_polygon(self, vertices, label=None, color='blue'):
        polygon = Polygon(vertices, closed=True, edgecolor='black', facecolor=color, label=label)
        self.ax.add_patch(polygon)

    def visualize(self):
        plt.show()

    def _on_scroll(self, event):
        if event.button == 'up':
            self.ax.set_xlim(self.ax.get_xlim()[0] * 0.9, self.ax.get_xlim()[1] * 0.9)
            self.ax.set_ylim(self.ax.get_ylim()[0] * 0.9, self.ax.get_ylim()[1] * 0.9)
        elif event.button == 'down':
            self.ax.set_xlim(self.ax.get_xlim()[0] * 1.1, self.ax.get_xlim()[1] * 1.1)
            self.ax.set_ylim(self.ax.get_ylim()[0] * 1.1, self.ax.get_ylim()[1] * 1.1)
        plt.draw()

# 测试用例
if __name__ == "__main__":
    # 创建可视化器对象
    visualizer = PolygonVisualizer()

    # 定义车辆的四边形参数
    car_x, car_y = 5, 5
    car_yaw = np.deg2rad(90)
    car_length, car_width = 4, 2

    # 定义障碍物的四边形参数
    obstacle_x, obstacle_y = 15, 15
    obstacle_yaw = np.deg2rad(60)
    obstacle_length, obstacle_width = 3, 1

    # 计算车辆和障碍物的四边形顶点
    car_vertices = visualizer._calculate_rectangle_vertices(car_x, car_y, car_length, car_width, car_yaw)
    obstacle_vertices = visualizer._calculate_rectangle_vertices(obstacle_x, obstacle_y, obstacle_length, obstacle_width, obstacle_yaw)

    # 可视化车辆和障碍物的四边形
    visualizer.visualize_polygon(car_vertices, label='Car', color='blue')
    visualizer.visualize_polygon(obstacle_vertices, label='Obstacle', color='red')

    # 显示图像
    visualizer.visualize()
