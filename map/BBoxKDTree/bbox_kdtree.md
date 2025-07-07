<!--
 * @Author: LOTEAT
 * @Date: 2025-07-07 10:52:14
-->
## BBoxKDTree详解

[知乎链接]()

[Github]()

本篇博客主要介绍一下apollo中实现的`AABoxKDTree2d`类。`KDTree`的原理比较简单，这里就不再介绍了，可以自行查阅资料了解。

### 1. AABoxKDTreeParams
```cpp
struct AABoxKDTreeParams {
  /// The maximum depth of the kdtree.
  int max_depth = -1;
  /// The maximum number of items in one leaf node.
  int max_leaf_size = -1;
  /// The maximum dimension size of leaf node.
  double max_leaf_dimension = -1.0;
};
```
`AABoxKDTreeParams` 结构体用于配置 AABoxKDTree2d（二维包围盒KD树）的构建和查询行为。主要参数如下：

- `max_depth`：KD树的最大深度。若为-1，则不限制深度。较大的深度可以提升查询精度，但会增加构建和查询的时间开销。
- `max_leaf_size`：每个叶子节点中最多包含的元素数量。若为-1，则不限制。该参数影响树的分裂策略，较小的值有助于提升查询效率，但会增加树的层数。
- `max_leaf_dimension`：叶子节点所包含元素的最大空间尺寸（如包围盒的最大边长）。若为-1.0，则不限制。该参数可用于控制空间划分的精细程度。

### 2. AABoxKDTree2dNode
`AABoxKDTree2dNode`是KDTree中的节点，里面储存的就是数据。先来看看几个辅助函数。

#### 2.1 ComputeBoundary

```cpp
  void ComputeBoundary(const std::vector<ObjectPtr> &objects) {
    min_x_ = std::numeric_limits<double>::infinity();
    min_y_ = std::numeric_limits<double>::infinity();
    max_x_ = -std::numeric_limits<double>::infinity();
    max_y_ = -std::numeric_limits<double>::infinity();
    for (ObjectPtr object : objects) {
      min_x_ = std::fmin(min_x_, object->aabox().min_x());
      max_x_ = std::fmax(max_x_, object->aabox().max_x());
      min_y_ = std::fmin(min_y_, object->aabox().min_y());
      max_y_ = std::fmax(max_y_, object->aabox().max_y());
    }
    mid_x_ = (min_x_ + max_x_) / 2.0;
    mid_y_ = (min_y_ + max_y_) / 2.0;
    ACHECK(!std::isinf(max_x_) && !std::isinf(max_y_) && !std::isinf(min_x_) &&
           !std::isinf(min_y_))
        << "the provided object box size is infinity";
  }
```
`ComputeBoundary` 是 `AABoxKDTree2dNode` 的辅助函数，用于计算当前节点所包含所有对象的整体空间边界（即最小外接矩形），同时也计算了中心点坐标，作为KDTree的起始点。

#### 2.2 ComputePartition

```cpp
  void ComputePartition() {
    if (max_x_ - min_x_ >= max_y_ - min_y_) {
      partition_ = PARTITION_X;
      partition_position_ = (min_x_ + max_x_) / 2.0;
    } else {
      partition_ = PARTITION_Y;
      partition_position_ = (min_y_ + max_y_) / 2.0;
    }
  }
```
`ComputePartition` 是 `AABoxKDTree2dNode` 的辅助函数，用于确定当前节点的空间划分方式。

- 其核心思想是：比较节点包围盒在 x 方向和 y 方向的跨度，选择跨度更大的方向作为划分轴。
- 如果 x 方向跨度更大，则以 x 轴为分割轴（PARTITION_X），分割位置为 (min_x + max_x) / 2；否则以 y 轴为分割轴（PARTITION_Y），分割位置为 (min_y + max_y) / 2。


#### 2.3 SplitToSubNodes

```cpp
  bool SplitToSubNodes(const std::vector<ObjectPtr> &objects,
                       const AABoxKDTreeParams &params) {
    if (params.max_depth >= 0 && depth_ >= params.max_depth) {
      return false;
    }
    if (static_cast<int>(objects.size()) <= std::max(1, params.max_leaf_size)) {
      return false;
    }
    if (params.max_leaf_dimension >= 0.0 &&
        std::max(max_x_ - min_x_, max_y_ - min_y_) <=
            params.max_leaf_dimension) {
      return false;
    }
    return true;
  }
```
这个就是判断是否要进行节点分裂，分裂条件就是设定好的配置参数，继续分裂就返回true，否则就返回false。


### 3. AABoxKDTree2d
```cpp
template <class ObjectType>
class AABoxKDTree2d {
 public:
  using ObjectPtr = const ObjectType *;

  /**
   * @brief Constructor which takes a vector of objects and parameters.
   * @param params Parameters to build the KD-tree.
   */
  AABoxKDTree2d(const std::vector<ObjectType> &objects,
                const AABoxKDTreeParams &params) {
    if (!objects.empty()) {
      std::vector<ObjectPtr> object_ptrs;
      for (const auto &object : objects) {
        object_ptrs.push_back(&object);
      }
      root_.reset(new AABoxKDTree2dNode<ObjectType>(object_ptrs, params, 0));
    }
  }

  /**
   * @brief Get the nearest object to a target point.
   * @param point The target point. Search it's nearest object.
   * @return The nearest object to the target point.
   */
  ObjectPtr GetNearestObject(const Vec2d &point) const {
    return root_ == nullptr ? nullptr : root_->GetNearestObject(point);
  }

  /**
   * @brief Get objects within a distance to a point.
   * @param point The center point of the range to search objects.
   * @param distance The radius of the range to search objects.
   * @return All objects within the specified distance to the specified point.
   */
  std::vector<ObjectPtr> GetObjects(const Vec2d &point,
                                    const double distance) const {
    if (root_ == nullptr) {
      return {};
    }
    return root_->GetObjects(point, distance);
  }

  /**
   * @brief Get the axis-aligned bounding box of the objects.
   * @return The axis-aligned bounding box of the objects.
   */
  AABox2d GetBoundingBox() const {
    return root_ == nullptr ? AABox2d() : root_->GetBoundingBox();
  }

 private:
  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> root_ = nullptr;
};
```