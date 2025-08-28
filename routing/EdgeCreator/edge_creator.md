<!--
 * @Author: LOTEAT
 * @Date: 2025-08-25 20:43:05
-->

## EdgeCreator详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/EdgeCreator/edge_creator.md)

### 1. Edge Creator
```cpp

void GetPbEdge(const Node& node_from, const Node& node_to,
               const Edge::DirectionType& type,
               const RoutingConfig& routing_config, Edge* edge) {
  edge->set_from_lane_id(node_from.lane_id());
  edge->set_to_lane_id(node_to.lane_id());
  edge->set_direction_type(type);

  edge->set_cost(0.0);
  if (type == Edge::LEFT || type == Edge::RIGHT) {
    const auto& target_range =
        (type == Edge::LEFT) ? node_from.left_out() : node_from.right_out();
    double changing_area_length = 0.0;
    for (const auto& range : target_range) {
      changing_area_length += range.end().s() - range.start().s();
    }
    double ratio = 1.0;
    if (changing_area_length < routing_config.base_changing_length()) {
      ratio = std::pow(
          changing_area_length / routing_config.base_changing_length(), -1.5);
    }
    edge->set_cost(routing_config.change_penalty() * ratio);
  }
}
```

`GetPbEdge` 函数负责创建拓扑图中连接两个节点的边，并根据边的类型和特性计算相应的代价值。该函数首先设置边的基本属性，包括源节点和目标节点的车道ID以及边的方向类型。对于前向边（FORWARD），代价设置为0；而对于变道边（LEFT或RIGHT），函数会计算可变道区域的总长度，通过遍历相应方向的可变道范围来累加长度。当可变道区域长度小于配置中的基础变道长度时，使用负1.5次幂的比率来增加代价，这种设计使得变道距离越短代价越高，从而鼓励在有足够距离的地方进行变道操作。最终的变道代价通过变道惩罚值与计算得出的比率相乘确定。

