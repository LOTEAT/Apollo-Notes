<!--
 * @Author: LOTEAT
 * @Date: 2025-08-06 15:02:49
-->

## NodeSRange详解

[知乎链接](https://zhuanlan.zhihu.com/p/1936724477841999629)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/NodeSRange/node_s_range.md)

在routing模块中，需要将原始的高精地图构建为拓扑地图，从而才能方便的执行导航规划。本篇博客主要介绍一下routing中的`NodeSRange`类。

### 1. Defination
```cpp
class NodeSRange {
 public:
  static bool IsEnoughForChangeLane(double start_s, double end_s);
  static bool IsEnoughForChangeLane(double length);

 public:
  NodeSRange() = default;
  NodeSRange(double s1, double s2);
  virtual ~NodeSRange() = default;

  bool operator<(const NodeSRange& other) const;
  bool IsValid() const;
  double StartS() const;
  double EndS() const;
  bool IsEnoughForChangeLane() const;
  double Length() const;

  void SetStartS(double start_s);
  void SetEndS(double end_s);
  bool MergeRangeOverlap(const NodeSRange& other);

 private:
  double start_s_ = 0.0;
  double end_s_ = 0.0;
};
```

`NodeSRange` 类是 Apollo 路径规划系统中用于表示车道节点的数据结构。在高精地图的拓扑结构构建过程中，每条车道都需要被抽象为图论中的节点。该类通过 `start_s_` 和 `end_s_` 两个成员变量定义了一个闭区间 [start_s, end_s]，其中 s 坐标表示沿车道中心线方向的距离。

`IsEnoughForChangeLane` 方法通过计算范围长度来判断当前区间是否为车辆提供了足够的变道距离。类中提供了两个重载版本的静态方法。`MergeRangeOverlap` 方法则处理了拓扑图构建过程中经常遇到的范围重叠问题，当两个相邻或重叠的车道段需要合并时，该方法能计算出合并后的有效范围。

### 2. IsEnoughForChangeLane
```cpp
bool NodeSRange::IsEnoughForChangeLane(double start_s, double end_s) {
  return IsEnoughForChangeLane(end_s - start_s);
}

bool NodeSRange::IsEnoughForChangeLane(double length) {
  return (length > FLAGS_min_length_for_lane_change);
}
```

`IsEnoughForChangeLane` 函数是 `NodeSRange` 类中用于评估车道变换安全性的方法，它通过比较给定的距离长度与系统配置的最小变道距离阈值来判断当前车道段是否提供了足够的空间供车辆安全完成变道操作。

### 3. NodeSRange
```cpp
NodeSRange::NodeSRange(double s1, double s2) : start_s_(s1), end_s_(s2) {}
```
构造函数。

### 4. operator<
```cpp
bool NodeSRange::operator<(const NodeSRange& other) const {
  return StartS() < other.StartS();
}
```
根据比较起始点判定大小关系。

### 5. MergeRangeOverlap
```cpp
bool NodeSRange::MergeRangeOverlap(const NodeSRange& other) {
  if (!IsValid() || !other.IsValid()) {
    return false;
  }
  if (other.StartS() > EndS() || other.EndS() < StartS()) {
    return false;
  }
  SetEndS(std::max(EndS(), other.EndS()));
  SetStartS(std::min(StartS(), other.StartS()));
  return true;
}
```
合并操作，当两段道路出现重叠后，将两段道路合并。

其余函数很简单，可自行阅读代码。