<!--
 * @Author: LOTEAT
 * @Date: 2025-08-06 15:16:34
-->
## TopoNode详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/TopoNode/topo_node.md)

### 1. Defination
```cpp
class TopoNode {
 public:
  static bool IsOutRangeEnough(const std::vector<NodeSRange>& range_vec,
                               double start_s, double end_s);

 public:
  explicit TopoNode(const Node& node);
  TopoNode(const TopoNode* topo_node, const NodeSRange& range);

  ~TopoNode();

  const Node& PbNode() const;
  double Length() const;
  double Cost() const;
  bool IsVirtual() const;

  const std::string& LaneId() const;
  const std::string& RoadId() const;
  const hdmap::Curve& CentralCurve() const;
  const common::PointENU& AnchorPoint() const;
  const std::vector<NodeSRange>& LeftOutRange() const;
  const std::vector<NodeSRange>& RightOutRange() const;

  const std::unordered_set<const TopoEdge*>& InFromAllEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromLeftOrRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromPreEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToAllEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToRightEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftOrRightEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToSucEdge() const;

  const TopoEdge* GetInEdgeFrom(const TopoNode* from_node) const;
  const TopoEdge* GetOutEdgeTo(const TopoNode* to_node) const;

  const TopoNode* OriginNode() const;
  double StartS() const;
  double EndS() const;
  bool IsSubNode() const;
  bool IsInFromPreEdgeValid() const;
  bool IsOutToSucEdgeValid() const;
  bool IsOverlapEnough(const TopoNode* sub_node,
                       const TopoEdge* edge_for_type) const;
  void AddInEdge(const TopoEdge* edge);
  void AddOutEdge(const TopoEdge* edge);

 private:
  void Init();
  bool FindAnchorPoint();
  void SetAnchorPoint(const common::PointENU& anchor_point);

  Node pb_node_;
  common::PointENU anchor_point_;

  double start_s_;
  double end_s_;
  bool is_left_range_enough_;
  int left_prefer_range_index_;
  bool is_right_range_enough_;
  int right_prefer_range_index_;

  std::vector<NodeSRange> left_out_sorted_range_;
  std::vector<NodeSRange> right_out_sorted_range_;

  std::unordered_set<const TopoEdge*> in_from_all_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_left_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_right_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_left_or_right_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_pre_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_all_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_left_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_right_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_left_or_right_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_suc_edge_set_;

  std::unordered_map<const TopoNode*, const TopoEdge*> out_edge_map_;
  std::unordered_map<const TopoNode*, const TopoEdge*> in_edge_map_;

  const TopoNode* origin_node_;
};
```

`TopoNode` 类是 Apollo 导航规划系统中拓扑图的节点表示，它将高精地图中的车道抽象为图论算法可处理的节点结构。该类不仅封装了车道的基本几何信息，还维护了复杂的拓扑连接关系，包括与相邻节点的入边和出边连接、左右变道的可达性分析、以及支持子节点等功能。要注意的是，在Apollo拓扑图中，车道就是节点，车道和车道之间的连接关系是边。

### 2. Curve Message
```protobuf
message CurvePoint {
  optional double s = 1;
}

message CurveRange {
  optional CurvePoint start = 1;
  optional CurvePoint end = 2;
}
```

`CurvePoint` 和 `CurveRange` 是 Apollo 导航规划系统中用于描述曲线几何信息的基础消息类型。`CurvePoint` 定义了曲线上的单个点位置，其中 `s` 参数表示沿曲线中心线的累积弧长距离。`CurveRange` 则通过起始点和结束点定义了曲线上的一个连续区间。

### 3. ConvertOutRange
```cpp
void ConvertOutRange(const RepeatedPtrField<CurveRange>& range_vec,
                     double start_s, double end_s,
                     std::vector<NodeSRange>* out_range, int* prefer_index) {
  out_range->clear();
  for (const auto& c_range : range_vec) {
    double s_s = c_range.start().s();
    double e_s = c_range.end().s();
    if (e_s < start_s || s_s > end_s || e_s < s_s) {
      continue;
    }
    s_s = std::max(start_s, s_s);
    e_s = std::min(end_s, e_s);
    NodeSRange s_range(s_s, e_s);
    out_range->push_back(std::move(s_range));
  }
  sort(out_range->begin(), out_range->end());
  int max_index = -1;
  double max_diff = 0.0;
  for (size_t i = 0; i < out_range->size(); ++i) {
    if (out_range->at(i).Length() > max_diff) {
      max_index = static_cast<int>(i);
      max_diff = out_range->at(i).Length();
    }
  }
  *prefer_index = max_index;
}
```

`ConvertOutRange` 函数负责将原始曲线范围数据转换为`NodeSRange`的工具函数。该函数接受一组 `CurveRange` 输入，通过与指定的起始和结束坐标进行交集计算，筛选出与当前节点空间范围重叠的有效区间，并将其转换为 `NodeSRange` 对象存储在输出向量中。函数还会对转换后的范围进行排序，并通过长度比较找出最长的范围区间作为首选索引。

### 4. IsOutRangeEnough
```cpp
bool TopoNode::IsOutRangeEnough(const std::vector<NodeSRange>& range_vec,
                                double start_s, double end_s) {
  if (!NodeSRange::IsEnoughForChangeLane(start_s, end_s)) {
    return false;
  }
  int start_index = BinarySearchForSLarger(range_vec, start_s);
  int end_index = BinarySearchForSSmaller(range_vec, end_s);

  int index_diff = end_index - start_index;
  if (start_index < 0 || end_index < 0) {
    return false;
  }
  if (index_diff > 1) {
    return true;
  }

  double pre_s_s = std::max(start_s, range_vec[start_index].StartS());
  double suc_e_s = std::min(end_s, range_vec[end_index].EndS());

  if (index_diff == 1) {
    double dlt = range_vec[start_index].EndS() - pre_s_s;
    dlt += suc_e_s - range_vec[end_index].StartS();
    return NodeSRange::IsEnoughForChangeLane(dlt);
  }
  if (index_diff == 0) {
    return NodeSRange::IsEnoughForChangeLane(pre_s_s, suc_e_s);
  }
  return false;
}
```

`IsOutRangeEnough` 函数是 `TopoNode` 类中用于判断指定范围内是否有足够空间进行变道操作的方法。该函数首先通过 `NodeSRange::IsEnoughForChangeLane` 对整体范围进行基础长度检查，然后使用二分查找算法在已排序的范围向量中定位与目标区间相交的起始和结束索引。

当 `index_diff > 1` 时，说明目标范围跨越了多个不相邻的可用区间，这种情况下有足够的空间保证变道安全，函数直接返回 `true`。当 `index_diff == 1` 时，表示目标范围恰好跨越两个相邻的可用区间，函数会分别计算这两个区间内的有效长度：第一个区间从有效起始位置到区间结束的长度，以及第二个区间从区间开始到有效结束位置的长度，然后将这两段长度相加判断是否满足变道所需的最小距离要求。

当 `index_diff == 0` 时，目标范围完全位于单一的可用区间内，函数会计算该区间与目标范围的交集部分，并判断这个交集长度是否足够支持变道操作。

### 5. Node Message
```protobuf
message Node {
  optional string lane_id = 1;
  optional double length = 2;
  repeated CurveRange left_out = 3;
  repeated CurveRange right_out = 4;
  optional double cost = 5;
  optional apollo.hdmap.Curve central_curve = 6;
  optional bool is_virtual = 7 [default = true];
  optional string road_id = 8;
}
```

`Node` 消息是拓扑图中节点的数据结构，它描述了一个车道节点的属性。`lane_id` 字段存储车道的标识符；`length` 字段记录车道的总长度；`left_out` 和 `right_out` 字段分别定义了该车道左侧和右侧的可变道范围，这些范围描述了车辆可以进行左变道或右变道的具体区间；`cost` 字段表示通过该车道的代价值，在路径搜索算法中用于评估路径的优劣；`central_curve` 字段包含车道中心线的详细几何信息，提供了车道的精确空间描述；`is_virtual` 字段标识该节点是否为虚拟节点，默认值为 true；`road_id` 字段存储该车道所属道路的标识符。


这里所出现的`apollo.hdmap.Curve`来自于`map_geometry.proto`:
```protobuf
// Generalization of a line.
message CurveSegment {
  oneof curve_type {
    LineSegment line_segment = 1;
  }
  optional double s = 6;  // start position (s-coordinate)
  optional apollo.common.PointENU start_position = 7;
  optional double heading = 8;  // start orientation
  optional double length = 9;
}

// An object similar to a line but that need not be straight.
message Curve {
  repeated CurveSegment segment = 1;
}
```

`CurveSegment` 和 `Curve` 是 Apollo 中用于描述复杂几何路径的消息类型。`CurveSegment` 表示曲线的基本构成单元，它通过 `oneof curve_type` 字段支持多种曲线类型（当前主要是线段），同时包含了该段曲线的起始位置参数：`s` 表示在整体曲线中的起始坐标，`start_position` 提供了三维空间中的起点位置，`heading` 定义了该段的起始方向角，`length` 记录了该段的长度。`Curve` 消息则通过 `repeated CurveSegment segment` 字段将多个曲线段有序组合，形成完整的复杂路径描述，这种分段式的设计能够精确表示现实世界中各种复杂的道路几何形状，包括直线段、弯道、坡道等。



### 6. 构造函数
```cpp
TopoNode::TopoNode(const Node& node)
    : pb_node_(node), start_s_(0.0), end_s_(pb_node_.length()) {
  ACHECK(pb_node_.length() > kLenghtEpsilon)
      << "Node length is invalid in pb: " << pb_node_.DebugString();
  Init();
  origin_node_ = this;
}

TopoNode::TopoNode(const TopoNode* topo_node, const NodeSRange& range)
    : TopoNode(topo_node->PbNode()) {
  origin_node_ = topo_node;
  start_s_ = range.StartS();
  end_s_ = range.EndS();
  Init();
}
```

`TopoNode` 类提供了两个构造函数。第一个构造函数 `TopoNode(const Node& node)` 是基础构造函数，用于从 protobuf 消息创建完整的拓扑节点，它接受一个 `Node` 消息对象并将其存储在 `pb_node_` 成员中，同时将起始坐标 `start_s_` 设置为 0，结束坐标 `end_s_` 设置为车道的总长度。构造函数会进行长度有效性检查，确保车道长度大于最小阈值 `kLenghtEpsilon`，随后调用 `Init()` 方法完成节点的初始化工作，并将 `origin_node_` 设置为自身，表明这是一个原始的完整节点。

第二个构造函数 `TopoNode(const TopoNode* topo_node, const NodeSRange& range)` 专门用于创建子节点或部分节点，它通过委托调用第一个构造函数来复用基础的初始化逻辑，但随后会重新设置关键的范围参数。该构造函数将 `origin_node_` 指向传入的原始节点，建立了子节点与原始节点之间的引用关系，这使得子节点可以追溯到其来源的完整节点，便于在路径规划过程中进行节点关系的管理和查询。同时，构造函数根据传入的 `NodeSRange` 对象重新设置 `start_s_` 和 `end_s_`，使得子节点只表示原始车道的一个特定区间。

### 7. Init
```cpp
void TopoNode::Init() {
  if (!FindAnchorPoint()) {
    AWARN << "Be attention!!! Find anchor point failed for lane: " << LaneId();
  }
  ConvertOutRange(pb_node_.left_out(), start_s_, end_s_,
                  &left_out_sorted_range_, &left_prefer_range_index_);

  is_left_range_enough_ =
      (left_prefer_range_index_ >= 0) &&
      left_out_sorted_range_[left_prefer_range_index_].IsEnoughForChangeLane();

  ConvertOutRange(pb_node_.right_out(), start_s_, end_s_,
                  &right_out_sorted_range_, &right_prefer_range_index_);
  is_right_range_enough_ = (right_prefer_range_index_ >= 0) &&
                           right_out_sorted_range_[right_prefer_range_index_]
                               .IsEnoughForChangeLane();
}
```
`Init` 函数是 `TopoNode` 类的初始化方法。函数首先调用 `FindAnchorPoint()` 方法来确定节点的锚点位置，如果锚点查找失败，函数会发出警告信息。

函数的主要工作集中在变道范围的处理上，它分别对左侧和右侧变道范围进行相同的处理流程。对于左侧变道，函数调用 `ConvertOutRange` 方法将原始的 `pb_node_.left_out()` 曲线范围数据转换为当前节点范围内的有效 `NodeSRange` 集合，并存储在 `left_out_sorted_range_` 中，同时获得首选范围的索引 `left_prefer_range_index_`。随后，函数通过检查首选索引的有效性（大于等于 0）以及首选范围的长度是否满足变道要求来设置 `is_left_range_enough_` 直接反映了该节点是否具备左变道的能力。

右侧变道的处理逻辑与左侧完全对应。

### 8. AnchorPoint
```cpp
bool TopoNode::FindAnchorPoint() {
  double total_size = 0;
  for (const auto& seg : CentralCurve().segment()) {
    total_size += seg.line_segment().point_size();
  }
  double rate = (StartS() + EndS()) / 2.0 / Length();
  int anchor_index = static_cast<int>(total_size * rate);
  for (const auto& seg : CentralCurve().segment()) {
    if (anchor_index < seg.line_segment().point_size()) {
      SetAnchorPoint(seg.line_segment().point(anchor_index));
      return true;
    }
    anchor_index -= seg.line_segment().point_size();
  }
  return false;
}

void TopoNode::SetAnchorPoint(const common::PointENU& anchor_point) {
  anchor_point_ = anchor_point;
}
```

`FindAnchorPoint`首先遍历车道中心线的所有曲线段，统计所有线段中包含的点的总数量。随后，函数计算中心位置节点在原始车道中的相对位置比例，通过 `(StartS() + EndS()) / 2.0 / Length()` 得到节点中心点相对于整个车道长度的比率。

函数接下来通过 `total_size * rate` 计算得到目标锚点在所有几何点中的索引位置。在确定索引后，函数再次遍历中心线的各个曲线段，通过累减索引值的方式定位到包含目标锚点的具体曲线段和该段内的精确点位置，一旦找到合适的点，函数立即调用 `SetAnchorPoint` 方法设置锚点并返回成功状态。`SetAnchorPoint` 函数的实现相对简单，它直接将传入的三维坐标点赋值给 `anchor_point_` 成员变量。


### 9. IsOutToSucEdgeValid
```cpp
bool TopoNode::IsOutToSucEdgeValid() const {
  return std::fabs(EndS() - OriginNode()->EndS()) < MIN_INTERNAL_FOR_NODE;
}
```

`IsOutToSucEdgeValid` 函数是用于验证当前节点是否具备有效后继边连接能力的方法。该函数通过比较当前节点的结束坐标 `EndS()` 与其原始节点的结束坐标 `OriginNode()->EndS()` 之间的距离差值来判断节点的边界位置特性，当两者之间的绝对差值小于预定义的最小内部阈值 `MIN_INTERNAL_FOR_NODE` 时，函数返回 `true`，表明当前节点位于原始车道的末端附近，具备向后续车道节点建立连接边的有效性。

### 10. IsInFromPreEdgeValid
```cpp
bool TopoNode::IsInFromPreEdgeValid() const {
  return std::fabs(StartS() - OriginNode()->StartS()) < MIN_INTERNAL_FOR_NODE;
}
```
`IsInFromPreEdgeValid`和`IsOutToSucEdgeValid` 类似，只不过是判断前驱是否具有连接能力。

### 10. AddInEdge
```cpp
void TopoNode::AddInEdge(const TopoEdge* edge) {
  if (edge->ToNode() != this) {
    return;
  }
  if (in_edge_map_.count(edge->FromNode()) != 0) {
    return;
  }
  switch (edge->Type()) {
    case TET_LEFT:
      in_from_right_edge_set_.insert(edge);
      in_from_left_or_right_edge_set_.insert(edge);
      break;
    case TET_RIGHT:
      in_from_left_edge_set_.insert(edge);
      in_from_left_or_right_edge_set_.insert(edge);
      break;
    default:
      in_from_pre_edge_set_.insert(edge);
      break;
  }
  in_from_all_edge_set_.insert(edge);
  in_edge_map_[edge->FromNode()] = edge;
}
```

`AddInEdge` 函数是 `TopoNode` 类中用于添加入边连接的方法，它负责维护节点的拓扑连接关系和边分类管理。函数首先进行有效性检查：确认传入的边确实指向当前节点（`edge->ToNode() != this`）以及验证来源节点尚未建立连接关系（避免重复边），只有通过这些检查的边才会被接受。接下来，函数根据边的类型进行分类存储，当边类型为 `TET_LEFT` 时，表示这是一条左变道边，函数将其添加到 `in_from_right_edge_set_` 和 `in_from_left_or_right_edge_set_` 中，这种看似反向的逻辑是因为左变道边意味着车辆从右侧车道变入当前车道；同样，`TET_RIGHT` 类型的右变道边被添加到 `in_from_left_edge_set_` 和 `in_from_left_or_right_edge_set_` 中，而其他类型的边（如直行连接）则被归类到 `in_from_pre_edge_set_` 中。最后，函数将所有入边统一添加到 `in_from_all_edge_set_` 中以便快速访问，并在 `in_edge_map_` 中建立从来源节点到边的映射关系。


### 11. AddOutEdge
```cpp
void TopoNode::AddOutEdge(const TopoEdge* edge) {
  if (edge->FromNode() != this) {
    return;
  }
  if (out_edge_map_.count(edge->ToNode()) != 0) {
    return;
  }
  switch (edge->Type()) {
    case TET_LEFT:
      out_to_left_edge_set_.insert(edge);
      out_to_left_or_right_edge_set_.insert(edge);
      break;
    case TET_RIGHT:
      out_to_right_edge_set_.insert(edge);
      out_to_left_or_right_edge_set_.insert(edge);
      break;
    default:
      out_to_suc_edge_set_.insert(edge);
      break;
  }
  out_to_all_edge_set_.insert(edge);
  out_edge_map_[edge->ToNode()] = edge;
}
```
逻辑上和`AddInEdge`是一样的，只是这里不需要把反向储存了。


