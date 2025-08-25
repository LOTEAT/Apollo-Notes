<!--
 * @Author: LOTEAT
 * @Date: 2025-08-25 16:34:22
-->

## Node Creator详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/NodeCreator/node_creator.md)

### 1. GetPbNode
```cpp
void GetPbNode(const hdmap::Lane& lane, const std::string& road_id,
               const RoutingConfig& routingconfig, Node* const node);
```
在`node_creator.h`中，定义了一个`GetPbNode`函数。通过高精地图的`lane`、`lane`所对应的道路id，以及导航配置文件，生成一个拓扑节点。

```cpp
void GetPbNode(const hdmap::Lane& lane, const std::string& road_id,
               const RoutingConfig& routingconfig, Node* const node) {
  InitNodeInfo(lane, road_id, node);
  InitNodeCost(lane, routingconfig, node);
}
```
其函数的定义也很简单，先初始化节点的信息，再初始化节点的cost，就可以获得一个拓扑节点。
### 2. InitNodeInfo
```cpp
void InitNodeInfo(const Lane& lane, const std::string& road_id,
                  Node* const node) {
  double lane_length = GetLaneLength(lane);
  node->set_lane_id(lane.id().id());
  node->set_road_id(road_id);
  AddOutBoundary(lane.left_boundary(), lane_length, node->mutable_left_out());
  AddOutBoundary(lane.right_boundary(), lane_length, node->mutable_right_out());
  node->set_length(lane_length);
  node->mutable_central_curve()->CopyFrom(lane.central_curve());
  node->set_is_virtual(true);
  if (!lane.has_junction_id() ||
      lane.left_neighbor_forward_lane_id_size() > 0 ||
      lane.right_neighbor_forward_lane_id_size() > 0) {
    node->set_is_virtual(false);
  }
}
```

`InitNodeInfo` 函数负责初始化拓扑节点的基本信息，将高精度地图中的车道数据转换为路径规划所需的节点属性。该函数首先计算车道长度并设置节点的车道ID和道路ID，然后通过 `AddOutBoundary` 函数处理车道的左右边界信息，将边界数据添加到节点的输出边界中。接下来设置节点的长度和中心曲线信息，并默认将节点标记为虚拟节点。最后通过判断车道是否属于交叉口或者是否存在左右相邻的前向车道来决定节点的虚拟属性：如果车道不在交叉口内或者存在相邻车道，则将节点标记为非虚拟节点。

### 3. GetLaneLength
```cpp
double GetLaneLength(const Lane& lane) {
  double length = 0.0;
  for (const auto& segment : lane.central_curve().segment()) {
    length += segment.length();
  }
  return length;
}
```

`GetLaneLength` 函数用于计算车道的总长度，通过遍历车道中心曲线的所有线段并累加各段长度来实现。该函数采用简单直接的累加方式，对车道中心曲线中的每个几何线段进行长度求和，最终返回整条车道的总长度。

### 4. GetLengthbyRate
```cpp
double GetLengthbyRate(double cur_s, double cur_total_length,
                       double target_length) {
  double new_length = cur_s / cur_total_length * target_length;
  return std::min(new_length, target_length);
}
```

`GetLengthbyRate` 函数用于按比例计算长度映射，实现从一个长度空间到另一个长度空间的坐标转换。该函数接收当前位置 `cur_s`、当前总长度 `cur_total_length` 和目标总长度 `target_length` 三个参数，通过比例计算得出在目标长度空间中对应的位置。计算公式 `cur_s / cur_total_length * target_length` 确保了位置的相对比例保持不变，而 `std::min` 函数则保证结果不会超过目标长度的边界。之所以有这个函数，是由于非直线车道的存在导致边界线和中心线长度不一致。因此，需要一种映射关系进行长度计算。


### 5. IsAllowedOut
```cpp
bool IsAllowedOut(const LaneBoundaryType& type) {
  if (type.types(0) == LaneBoundaryType::DOTTED_YELLOW ||
      type.types(0) == LaneBoundaryType::DOTTED_WHITE) {
    return true;
  }
  return false;
}
```

`IsAllowedOut` 函数用于判断车道边界是否允许车辆穿越进行变道操作。该函数检查车道边界类型的第一个类型属性，只有当边界类型为虚线黄线（DOTTED_YELLOW）或虚线白线（DOTTED_WHITE）时才返回 true，表示允许跨越该边界。

### 6. AddOutBoundary
```cpp
void AddOutBoundary(const LaneBoundary& bound, double lane_length,
                    RepeatedPtrField<CurveRange>* const out_range) {
  for (int i = 0; i < bound.boundary_type_size(); ++i) {
    if (!IsAllowedOut(bound.boundary_type(i))) {
      continue;
    }
    CurveRange* range = out_range->Add();
    range->mutable_start()->set_s(GetLengthbyRate(bound.boundary_type(i).s(),
                                                  bound.length(), lane_length));
    if (i != bound.boundary_type_size() - 1) {
      range->mutable_end()->set_s(GetLengthbyRate(
          bound.boundary_type(i + 1).s(), bound.length(), lane_length));
    } else {
      range->mutable_end()->set_s(lane_length);
    }
  }
}
```

`AddOutBoundary` 函数负责处理车道边界信息并生成可变道区域范围。该函数遍历车道边界的所有边界类型，通过 `IsAllowedOut` 函数筛选出允许跨越的边界段（虚线区域），然后为每个可变道段创建一个 `CurveRange` 对象。函数使用 `GetLengthbyRate` 进行长度映射，将边界坐标系中的位置转换为车道坐标系中对应的位置。对于每个有效的边界段，函数设置其起始位置为当前边界类型的起始位置，结束位置为下一个边界类型的起始位置，而对于最后一个边界段则以车道总长度作为结束位置。

### 7. RoutingConfig
```protobuf
message TopicConfig {
  optional string routing_response_topic = 1;
  optional string routing_response_history_topic = 2;
}

message RoutingConfig {
  optional double base_speed = 1;  // base speed for node creator [m/s]
  optional double left_turn_penalty =
      2;  // left turn penalty for node creator [m]
  optional double right_turn_penalty =
      3;                              // right turn penalty for node creator [m]
  optional double uturn_penalty = 4;  // left turn penalty for node creator [m]
  optional double change_penalty = 5;  // change penalty for edge creator [m]
  optional double base_changing_length =
      6;  // base change length penalty for edge creator [m]
  optional TopicConfig topic_config = 7;
}
```

`RoutingConfig` 是路径规划系统的配置消息，定义了影响路径代价计算和规划行为的参数。该配置包含了基础速度 `base_speed` 用于设定默认行驶速度，以及各种转向惩罚参数：`left_turn_penalty`、`right_turn_penalty` 和 `uturn_penalty` 分别对应左转、右转和掉头的代价惩罚，这些参数通过增加相应动作的代价来影响路径选择偏好。变道相关的参数包括 `change_penalty` 和 `base_changing_length`，前者设定变道操作的基础惩罚，后者定义变道的基础长度惩罚，这些参数共同控制着路径规划算法对变道行为的偏好程度。此外，配置还包含 `TopicConfig` 用于设定话题通信参数。


### 8. InitNodeCost
```cpp
void InitNodeCost(const Lane& lane, const RoutingConfig& routing_config,
                  Node* const node) {
  double lane_length = GetLaneLength(lane);
  double speed_limit =
      lane.has_speed_limit() ? lane.speed_limit() : routing_config.base_speed();
  double ratio = speed_limit >= routing_config.base_speed()
                     ? std::sqrt(routing_config.base_speed() / speed_limit)
                     : 1.0;
  double cost = lane_length * ratio;
  if (lane.has_turn()) {
    if (lane.turn() == Lane::LEFT_TURN) {
      cost += routing_config.left_turn_penalty();
    } else if (lane.turn() == Lane::RIGHT_TURN) {
      cost += routing_config.right_turn_penalty();
    } else if (lane.turn() == Lane::U_TURN) {
      cost += routing_config.uturn_penalty();
    }
  }
  node->set_cost(cost);
}
```
`InitNodeCost` 函数负责计算和设置拓扑节点的代价值，这是路径规划算法中用于评估路径优劣的指标。该函数首先获取车道的限速信息，如果车道没有设定限速则使用配置中的基础速度作为默认值。接下来计算速度比率，当限速大于等于基础速度时，使用基础速度与限速比值的平方根（使得调节因子变化更平滑）作为调节因子，否则设为1.0，这种设计使得高速道路具有更低的代价。基础代价通过车道长度与速度比率的乘积计算得出，然后根据车道的转向类型添加相应的转向惩罚：左转、右转和掉头分别对应不同的惩罚值。这种代价计算策略综合考虑了距离、速度限制和转向复杂度等因素，使得路径规划算法能够优先选择距离短、速度快且转向简单的路径。
