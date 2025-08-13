<!--
 * @Author: LOTEAT
 * @Date: 2025-07-07 23:55:57
-->

## LineSegment2d详解

[知乎链接]()

[Github]()

本篇博客主要介绍一下apollo中实现的`LineSegment2d`类。这个类对一段segment的line进行了封装。

### 1. LineSegment2d

该类的核心接口如下：

#### 1.1 Constructor

```cpp
LineSegment2d::LineSegment2d() { unit_direction_ = Vec2d(1, 0); }

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();
}
```

`LineSegment2d`提供了两个构造函数：默认构造函数创建一个单位方向向量为(1,0)的空线段；参数构造函数接受起点和终点，自动计算线段长度、单位方向向量和朝向角，其中单位方向向量通过归一化处理得到，当线段长度小于数学epsilon时设为零向量以避免数值问题。

#### 1.2 rotate

```cpp
Vec2d LineSegment2d::rotate(const double angle) {
  Vec2d diff_vec = end_ - start_;
  diff_vec.SelfRotate(angle);
  return start_ + diff_vec;
}
```

`rotate`函数以起点为中心旋转线段指定角度并返回新的终点位置：首先计算从起点到终点的向量，然后将此向量绕原点旋转指定角度，最后加上起点坐标得到旋转后的终点。这个函数保持起点不变，只改变线段的方向。

### 1.3 DistanceTo

```cpp
double LineSegment2d::DistanceTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceTo(const Vec2d &point,
                                 Vec2d *const nearest_pt) const {
  CHECK_NOTNULL(nearest_pt);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}
```

`DistanceTo`函数提供了两个重载版本来计算点到线段的最短距离：

**第一个版本**只返回距离值，通过投影计算判断最近点位置：

-   若线段长度为0，返回点到起点的距离
-   若投影在起点前（proj ≤ 0），返回点到起点的距离
-   若投影在终点后（proj ≥ length），返回点到终点的距离
-   否则返回点到线段的垂直距离

**第二个版本**除了返回距离，还通过`nearest_pt`参数输出最近点坐标，算法逻辑相同但会设置对应的最近点位置（起点、终点或线段上的投影点）。

### 1.4 DistanceSquareTo

```cpp
double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point,
                                       Vec2d *const nearest_pt) const {
  CHECK_NOTNULL(nearest_pt);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}
```

这里的计算逻辑和`DistanceTo`是一样的，只不过最终是计算距离的平方。

### 1.5 IsPointIn

```cpp
bool LineSegment2d::IsPointIn(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= kMathEpsilon;
  }
  const double prod = CrossProd(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}
```

使用叉乘计算，判断点是否在线段上。如果叉乘结果接近0，就认为处于线段中。

### 1.6 ProjectOntoUnit

```cpp
double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
  return unit_direction_.InnerProd(point - start_);
}
```

`ProjectOntoUnit`计算点在线段方向上的投影长度：通过将从起点到目标点的向量与单位方向向量做内积，得到点在线段延长线上的投影距离，正值表示在线段方向上，负值表示在相反方向上。

### 1.7 ProductOntoUnit

```cpp
double LineSegment2d::ProductOntoUnit(const Vec2d &point) const {
  return unit_direction_.CrossProd(point - start_);
}
```

`ProductOntoUnit`计算点相对于线段的侧向偏移：通过将从起点到目标点的向量与单位方向向量做叉积，得到点到线段的有向距离，正值表示在线段左侧，负值表示在右侧，零值表示在线段上。

### 1.8 GetIntersect

```cpp
bool LineSegment2d::GetIntersect(const LineSegment2d &other_segment,
                                 Vec2d *const point) const {
  CHECK_NOTNULL(point);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kMathEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kMathEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}
```

`GetIntersect`函数计算两个线段的交点，采用分层检测策略：首先检查端点是否在对方线段上（快速情况），然后使用叉积判断两线段是否相交。算法通过计算每条线段两端点相对于另一线段的叉积符号来判断是否跨越，如果两个线段互相跨越则存在交点，最后通过比例插值计算具体交点坐标。

### 1.9 GetPerpendicularFoot

```cpp
double LineSegment2d::GetPerpendicularFoot(const Vec2d &point,
                                           Vec2d *const foot_point) const {
  CHECK_NOTNULL(foot_point);
  if (length_ <= kMathEpsilon) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}
```

`GetPerpendicularFoot`函数计算点到线段延长线的垂足并返回垂直距离：通过投影计算得到垂足在线段延长线上的位置，输出垂足坐标到`foot_point`参数中。与`DistanceTo`不同的是，该函数不限制垂足必须在线段范围内，而是允许垂足落在线段的延长线上。



Reference:

