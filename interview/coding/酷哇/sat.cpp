// Copyright (c) 2024, Unity-Drive Inc. All rights reserved.
// Author: Yu Chen chenyu@unity-drive.com
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

// 定义二维向量结构
struct Vector2 {
  double x, y;

  Vector2 operator-(const Vector2& other) const {
    return {x - other.x, y - other.y};
  }

  Vector2 operator+(const Vector2& other) const {
    return {x + other.x, y + other.y};
  }

  Vector2 operator*(double scalar) const { return {x * scalar, y * scalar}; }

  double dot(const Vector2& other) const { return x * other.x + y * other.y; }

  double norm() const { return std::sqrt(x * x + y * y); }

  Vector2 normalized() const {
    double n = norm();
    return {x / n, y / n};
  }
};

// 投影多边形
pair<double, double> projectPolygon(const vector<Vector2>& vertices,
                                    const Vector2& axis) {
  vector<double> dots;
  for (const auto& vertex : vertices) {
    dots.push_back(vertex.dot(axis));
  }
  double min_proj = *min_element(dots.begin(), dots.end());
  double max_proj = *max_element(dots.begin(), dots.end());
  return {min_proj, max_proj};
}

// 投影圆
pair<double, double> projectCircle(const Vector2& circle_center, double radius,
                                   const Vector2& axis) {
  double center_proj = circle_center.dot(axis);
  return {center_proj - radius, center_proj + radius};
}

// 分离轴定理检测
bool separatingAxisTheorem(const vector<Vector2>& vertices,
                           const Vector2& circle_center, double radius) {
  vector<Vector2> axes;
  int num_vertices = vertices.size();

  // 计算多边形边的法向量
  for (int i = 0; i < num_vertices; ++i) {
    Vector2 edge =
        vertices[i] - vertices[(i + num_vertices - 1) % num_vertices];
    Vector2 normal = {-edge.y, edge.x};
    axes.push_back(normal.normalized());
  }

  // 添加从圆心到多边形顶点的向量
  for (const auto& vertex : vertices) {
    Vector2 axis = vertex - circle_center;
    if (axis.norm() > 0) {
      axes.push_back(axis.normalized());
    }
  }

  // 投影并检查是否有分离轴
  for (const auto& axis : axes) {
    auto polygon_proj = projectPolygon(vertices, axis);
    auto circle_proj = projectCircle(circle_center, radius, axis);

    if (polygon_proj.second < circle_proj.first ||
        circle_proj.second < polygon_proj.first) {
      return false;
    }
  }

  return true;
}

int main() {
  // 定义矩形顶点（定向矩形）
  double angle = M_PI / 4;  // 45 degrees in radians
  double cos_angle = std::cos(angle);
  double sin_angle = std::sin(angle);
  Vector2 rect_center = {2, 2};
  Vector2 rect_size = {2, 2};
  Vector2 half_size = rect_size * 0.5;

  vector<Vector2> vertices = {{-half_size.x, -half_size.y},
                              {half_size.x, -half_size.y},
                              {half_size.x, half_size.y},
                              {-half_size.x, half_size.y}};

  vector<Vector2> rotated_vertices;
  for (const auto& vertex : vertices) {
    Vector2 rotated_vertex = {vertex.x * cos_angle - vertex.y * sin_angle,
                              vertex.x * sin_angle + vertex.y * cos_angle};
    rotated_vertices.push_back(rotated_vertex + rect_center);
  }

  // 定义圆
  Vector2 circle_center = {0, 1};
  double radius = std::sqrt(2);

  // 检查是否重叠
  bool is_overlapping =
      separatingAxisTheorem(rotated_vertices, circle_center, radius);
  cout << "Circle and oriented rectangle overlapping: "
       << (is_overlapping ? "true" : "false") << endl;
  return 0;
}
