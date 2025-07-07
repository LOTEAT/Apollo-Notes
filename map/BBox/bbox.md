<!--
 * @Author: LOTEAT
 * @Date: 2025-07-07 10:35:57
-->
## BBox详解

[知乎链接](https://zhuanlan.zhihu.com/p/1925506308330723138)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/map/BBox/bbox.md)

本篇博客主要介绍一下apollo中实现的`AABox2d`类。

### 1. AABox2d

`AABox2d` 是 Apollo 中用于表示二维轴对齐包围盒（Axis-Aligned Bounding Box, AABB）的核心类。它主要用于描述和计算地图元素、障碍物等在二维平面上的空间范围。ABBox的特点是，包围盒的边始终与坐标轴平行，便于快速判断点或其他盒体是否在范围内。


下方为`AABox2d`的主要接口与实现说明：

```cpp
class AABox2d {
 public:
  /**
   * @brief Default constructor.
   * Creates an axes-aligned box with zero length and width at the origin.
   */
  AABox2d() = default;
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box with given center, length, and width.
   * @param center The center of the box
   * @param length The size of the box along the x-axis
   * @param width The size of the box along the y-axis
   */
  AABox2d(const Vec2d &center, const double length, const double width);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box from two opposite corners.
   * @param one_corner One corner of the box
   * @param opposite_corner The opposite corner to the first one
   */
  AABox2d(const Vec2d &one_corner, const Vec2d &opposite_corner);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(const std::vector<Vec2d> &points);

  /**
   * @brief Getter of center_
   * @return Center of the box
   */
  const Vec2d &center() const { return center_; }

  /**
   * @brief Getter of x-component of center_
   * @return x-component of the center of the box
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of y-component of center_
   * @return y-component of the center of the box
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of length_
   * @return The length of the box
   */
  double length() const { return length_; }

  /**
   * @brief Getter of width_
   * @return The width of the box
   */
  double width() const { return width_; }

  /**
   * @brief Getter of half_length_
   * @return Half of the length of the box
   */
  double half_length() const { return half_length_; }

  /**
   * @brief Getter of half_width_
   * @return Half of the width of the box
   */
  double half_width() const { return half_width_; }

  /**
   * @brief Getter of length_*width_
   * @return The area of the box
   */
  double area() const { return length_ * width_; }

  /**
   * @brief Returns the minimum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double min_x() const { return center_.x() - half_length_; }

  /**
   * @brief Returns the maximum x-coordinate of the box
   *
   * @return x-coordinate
   */
  double max_x() const { return center_.x() + half_length_; }

  /**
   * @brief Returns the minimum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double min_y() const { return center_.y() - half_width_; }

  /**
   * @brief Returns the maximum y-coordinate of the box
   *
   * @return y-coordinate
   */
  double max_y() const { return center_.y() + half_width_; }

  /**
   * @brief Gets all corners in counter clockwise order.
   *
   * @param corners Output where the corners are written
   */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  /**
   * @brief Determines whether a given point is in the box.
   *
   * @param point The point we wish to test for containment in the box
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Determines whether a given point is on the boundary of the box.
   *
   * @param point The point we wish to test for boundary membership
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Determines the distance between a point and the box.
   *
   * @param point The point whose distance to the box we wish to determine.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Determines the distance between two boxes.
   *
   * @param box Another box.
   */
  double DistanceTo(const AABox2d &box) const;

  /**
   * @brief Determines whether two boxes overlap.
   *
   * @param box Another box
   */
  bool HasOverlap(const AABox2d &box) const;

  /**
   * @brief Shift the center of AABox by the input vector.
   *
   * @param shift_vec The vector by which we wish to shift the box
   */
  void Shift(const Vec2d &shift_vec);

  /**
   * @brief Changes box to include another given box, as well as the current
   * one.
   *
   * @param other_box Another box
   */
  void MergeFrom(const AABox2d &other_box);

  /**
   * @brief Changes box to include a given point, as well as the current box.
   *
   * @param other_point Another point
   */
  void MergeFrom(const Vec2d &other_point);

  /**
   * @brief Gets a human-readable debug string
   *
   * @return A string
   */
  std::string DebugString() const;

 private:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
};
```

### AABox2d类主要接口说明

- **构造函数**
  - `AABox2d()`：默认构造函数，创建一个以原点为中心、长宽为0的包围盒。
  - `AABox2d(const Vec2d &center, double length, double width)`：通过中心点、长、宽构造包围盒。
  - `AABox2d(const Vec2d &one_corner, const Vec2d &opposite_corner)`：通过对角线的两个点构造包围盒。
  - `explicit AABox2d(const std::vector<Vec2d> &points)`：通过包含所有点的最小包围盒构造。

- **属性访问**
  - `center()` / `center_x()` / `center_y()`：获取包围盒中心点及其坐标。
  - `length()` / `width()`：获取包围盒的长和宽。
  - `half_length()` / `half_width()`：获取长宽的一半。
  - `area()`：获取包围盒面积。
  - `min_x()` / `max_x()` / `min_y()` / `max_y()`：获取包围盒的边界坐标。

- **几何操作**
  - `GetAllCorners()`：获取包围盒四个角点，逆时针顺序。
  - `IsPointIn()`：判断点是否在包围盒内。
  - `IsPointOnBoundary()`：判断点是否在包围盒边界上。
  - `DistanceTo(const Vec2d &point)`：计算点到包围盒的最短距离。
  - `DistanceTo(const AABox2d &box)`：计算两个包围盒之间的最短距离。
  - `HasOverlap(const AABox2d &box)`：判断两个包围盒是否有重叠。
  - `Shift(const Vec2d &shift_vec)`：将包围盒整体平移。
  - `MergeFrom(const AABox2d &other_box)`：合并另一个包围盒，扩展当前包围盒以包含对方。
  - `MergeFrom(const Vec2d &other_point)`：合并一个点，扩展当前包围盒以包含该点。
  - `DebugString()`：输出包围盒的调试信息字符串。

这些接口为地图元素的空间计算、碰撞检测、可视化等提供了基础能力。

`AABox2d`的具体实现可以看看Apollo中的`aabox2d.cc`文件，内容也十分简单，这里就不再赘述。
