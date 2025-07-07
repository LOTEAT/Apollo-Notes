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

#### 2.4 InitObjects
```cpp
  void InitObjects(const std::vector<ObjectPtr> &objects) {
    num_objects_ = static_cast<int>(objects.size());
    objects_sorted_by_min_ = objects;
    objects_sorted_by_max_ = objects;
    std::sort(objects_sorted_by_min_.begin(), objects_sorted_by_min_.end(),
              [&](ObjectPtr obj1, ObjectPtr obj2) {
                return partition_ == PARTITION_X
                           ? obj1->aabox().min_x() < obj2->aabox().min_x()
                           : obj1->aabox().min_y() < obj2->aabox().min_y();
              });
    std::sort(objects_sorted_by_max_.begin(), objects_sorted_by_max_.end(),
              [&](ObjectPtr obj1, ObjectPtr obj2) {
                return partition_ == PARTITION_X
                           ? obj1->aabox().max_x() > obj2->aabox().max_x()
                           : obj1->aabox().max_y() > obj2->aabox().max_y();
              });
    objects_sorted_by_min_bound_.reserve(num_objects_);
    for (ObjectPtr object : objects_sorted_by_min_) {
      objects_sorted_by_min_bound_.push_back(partition_ == PARTITION_X
                                                 ? object->aabox().min_x()
                                                 : object->aabox().min_y());
    }
    objects_sorted_by_max_bound_.reserve(num_objects_);
    for (ObjectPtr object : objects_sorted_by_max_) {
      objects_sorted_by_max_bound_.push_back(partition_ == PARTITION_X
                                                 ? object->aabox().max_x()
                                                 : object->aabox().max_y());
    }
  }
```
这个函数读起来很简单，就是根据划分方向，进行数组的排序。稍微有些令人费解的是，对于左上角的点，排序是从小到大排序的。对于右下角的点，排序则是从大到小的。其实我这里的理解是，同时使用升序和降序也是可以的，只不过可能为了美观或者逻辑的顺畅，这里采用了两种排序方式。

#### 2.5 PartitionObjects
```cpp
  void PartitionObjects(const std::vector<ObjectPtr> &objects,
                        std::vector<ObjectPtr> *const left_subnode_objects,
                        std::vector<ObjectPtr> *const right_subnode_objects) {
    left_subnode_objects->clear();
    right_subnode_objects->clear();
    std::vector<ObjectPtr> other_objects;
    if (partition_ == PARTITION_X) {
      for (ObjectPtr object : objects) {
        if (object->aabox().max_x() <= partition_position_) {
          left_subnode_objects->push_back(object);
        } else if (object->aabox().min_x() >= partition_position_) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    } else {
      for (ObjectPtr object : objects) {
        if (object->aabox().max_y() <= partition_position_) {
          left_subnode_objects->push_back(object);
        } else if (object->aabox().min_y() >= partition_position_) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    }
    InitObjects(other_objects);
  }
```
分割节点的函数读起来很简单，如果物品的最大y值或者最大x值小于分割边界，那么就归于左节点；如果最小y值或者最小x值大于分割边界，那么就归于右节点。否则，物品就跨界了，这个时候并不会将其归于左节点或者右节点，而是将其储存在`other_objects`中，然后调用`InitObjects`。

注意，该函数将object中的位置进行排序，并储存到了成员变量中。所以这里本质上，就是保存成中间节点了。使用`InitObjects`对位置进行了排序，是方便未来在中间节点遍历时的处理。

#### 2.6 Constructor: AABoxKDTree2dNode
```cpp
  AABoxKDTree2dNode(const std::vector<ObjectPtr> &objects,
                    const AABoxKDTreeParams &params, int depth)
      : depth_(depth) {
    ACHECK(!objects.empty());

    ComputeBoundary(objects);
    ComputePartition();

    if (SplitToSubNodes(objects, params)) {
      std::vector<ObjectPtr> left_subnode_objects;
      std::vector<ObjectPtr> right_subnode_objects;
      PartitionObjects(objects, &left_subnode_objects, &right_subnode_objects);

      // Split to sub-nodes.
      if (!left_subnode_objects.empty()) {
        left_subnode_.reset(new AABoxKDTree2dNode<ObjectType>(
            left_subnode_objects, params, depth + 1));
      }
      if (!right_subnode_objects.empty()) {
        right_subnode_.reset(new AABoxKDTree2dNode<ObjectType>(
            right_subnode_objects, params, depth + 1));
      }
    } else {
      InitObjects(objects);
    }
  }
```
现在再来看这个构造函数就很简单了。首先计算边界和分割方向，这类似与初始化的过程。然后判断是否要继续分割节点，如果不分割，说明所有节点都属于中间节点，那么就调用`InitObjects`；否则，就对左右节点进行分割，然后左右节点递归调用构造函数，这就实现了整个KDTree的初始化。

#### 2.7 LowerDistanceSquareToPoint
```cpp
  double LowerDistanceSquareToPoint(const Vec2d &point) const {
    double dx = 0.0;
    if (point.x() < min_x_) {
      dx = min_x_ - point.x();
    } else if (point.x() > max_x_) {
      dx = point.x() - max_x_;
    }
    double dy = 0.0;
    if (point.y() < min_y_) {
      dy = min_y_ - point.y();
    } else if (point.y() > max_y_) {
      dy = point.y() - max_y_;
    }
    return dx * dx + dy * dy;
  }
```
这个逻辑很简单，如果这个点在大的bbox里，那么距离就是0；如果x在大的bbox里但是y不在，那么y的距离就是最小距离；反之，x的距离就是最小距离。

#### 2.8 UpperDistanceSquareToPoint

```cpp
  double UpperDistanceSquareToPoint(const Vec2d &point) const {
    const double dx =
        (point.x() > mid_x_ ? (point.x() - min_x_) : (point.x() - max_x_));
    const double dy =
        (point.y() > mid_y_ ? (point.y() - min_y_) : (point.y() - max_y_));
    return dx * dx + dy * dy;
  }
```
这个也很简单。沿中点横一刀竖一刀将这个世界切割成四份，bbox中离当前所处的位置最远的点就是当前位置的对向部分中的bbox顶点位置。

#### 2.9 GetAllObjects
```cpp
  void GetAllObjects(std::vector<ObjectPtr> *const result_objects) const {
    result_objects->insert(result_objects->end(),
                           objects_sorted_by_min_.begin(),
                           objects_sorted_by_min_.end());
    if (left_subnode_ != nullptr) {
      left_subnode_->GetAllObjects(result_objects);
    }
    if (right_subnode_ != nullptr) {
      right_subnode_->GetAllObjects(result_objects);
    }
  }
```
获取所有的节点。

#### 2.10 GetObjectsInternal
```cpp
  void GetObjectsInternal(const Vec2d &point, const double distance,
                          const double distance_sqr,
                          std::vector<ObjectPtr> *const result_objects) const {
    if (LowerDistanceSquareToPoint(point) > distance_sqr) {
      return;
    }
    if (UpperDistanceSquareToPoint(point) <= distance_sqr) {
      GetAllObjects(result_objects);
      return;
    }
    const double pvalue = (partition_ == PARTITION_X ? point.x() : point.y());
    if (pvalue < partition_position_) {
      const double limit = pvalue + distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_sorted_by_min_bound_[i] > limit) {
          break;
        }
        ObjectPtr object = objects_sorted_by_min_[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    } else {
      const double limit = pvalue - distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_sorted_by_max_bound_[i] < limit) {
          break;
        }
        ObjectPtr object = objects_sorted_by_max_[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    }
    if (left_subnode_ != nullptr) {
      left_subnode_->GetObjectsInternal(point, distance, distance_sqr,
                                        result_objects);
    }
    if (right_subnode_ != nullptr) {
      right_subnode_->GetObjectsInternal(point, distance, distance_sqr,
                                         result_objects);
    }
  }
```
这里要注意的是，distance是distance_sqr正平方根，我认为这里只是为了加速运算，所以才把两个值都传入进来，理论上传一个值就够了。

`GetObjectsInternal` 是 AABoxKDTree2dNode 的查询函数，用于在 KD 树结构中查找距离指定点一定范围内的所有对象。其基本思路是：首先通过节点包围盒与目标点的距离关系，快速判断当前节点是否有必要进一步遍历；如果整个节点都在查询范围外则直接返回，如果整个节点都在范围内则直接收集所有对象，否则根据分割轴和分割位置，结合对象的空间分布，对左右子节点递归查找，并对当前节点的对象集合进行距离过滤。

这里以对X轴分割为例，如果该点分布在左侧节点，那么其他bbox的min_x距离该点x坐标最多不能超过p_value + distance。反之，如果分布在右侧节点，那么limit的值就是p_value - distance。这其实就是一个剪枝操作，是为了性能设计的。而且，这里也可以看到先前`InitObjects`为什么要采用两个相反的顺序排序，就是为了这里逻辑处理的统一。


#### 2.11 GetNearestObjectInternal
```cpp
  void GetNearestObjectInternal(const Vec2d &point,
                                double *const min_distance_sqr,
                                ObjectPtr *const nearest_object) const {
    if (LowerDistanceSquareToPoint(point) >= *min_distance_sqr - kMathEpsilon) {
      return;
    }
    const double pvalue = (partition_ == PARTITION_X ? point.x() : point.y());
    const bool search_left_first = (pvalue < partition_position_);
    if (search_left_first) {
      if (left_subnode_ != nullptr) {
        left_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                nearest_object);
      }
    } else {
      if (right_subnode_ != nullptr) {
        right_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                 nearest_object);
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }

    if (search_left_first) {
      for (int i = 0; i < num_objects_; ++i) {
        const double bound = objects_sorted_by_min_bound_[i];
        if (bound > pvalue && Square(bound - pvalue) > *min_distance_sqr) {
          break;
        }
        ObjectPtr object = objects_sorted_by_min_[i];
        const double distance_sqr = object->DistanceSquareTo(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    } else {
      for (int i = 0; i < num_objects_; ++i) {
        const double bound = objects_sorted_by_max_bound_[i];
        if (bound < pvalue && Square(bound - pvalue) > *min_distance_sqr) {
          break;
        }
        ObjectPtr object = objects_sorted_by_max_[i];
        const double distance_sqr = object->DistanceSquareTo(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }
    if (search_left_first) {
      if (right_subnode_ != nullptr) {
        right_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                 nearest_object);
      }
    } else {
      if (left_subnode_ != nullptr) {
        left_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                nearest_object);
      }
    }
  }
```


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