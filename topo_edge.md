<!--
 * @Author: LOTEAT
 * @Date: 2025-08-12 19:15:34
-->

## TopoEdge详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/TopoEdge/topo_edge.md)

### 1. Defination

```cpp
class TopoEdge {
 public:
  TopoEdge(const Edge& edge, const TopoNode* from_node,
           const TopoNode* to_node);

  ~TopoEdge();

  const Edge& PbEdge() const;
  double Cost() const;
  const std::string& FromLaneId() const;
  const std::string& ToLaneId() const;
  TopoEdgeType Type() const;

  const TopoNode* FromNode() const;
  const TopoNode* ToNode() const;

 private:
  Edge pb_edge_;
  const TopoNode* from_node_ = nullptr;
  const TopoNode* to_node_ = nullptr;
};
```

`TopoEdge` 类是 Apollo 中拓扑图的边表示，它定义了车道节点之间的连接关系和通行方式。该类封装了原始的 `Edge` protobuf 消息以及连接的起始和目标节点指针，通过 `from_node_` 和 `to_node_` 成员变量建立了有向连接关系。

### 2. Edge Message

```protobuf
message Edge {
  enum DirectionType {
    FORWARD = 0;
    LEFT = 1;
    RIGHT = 2;
  }

  optional string from_lane_id = 1;
  optional string to_lane_id = 2;
  optional double cost = 3;
  optional DirectionType direction_type = 4;
}

```

`Edge` 消息是 Apollo 拓扑图中边连接的数据结构，它描述了两个车道节点之间的连接关系和通行属性。`from_lane_id` 和 `to_lane_id` 字段分别标识连接的起始车道和目标车道。`cost` 字段定义了通过该连接的代价值。`direction_type` 字段定义了连接的方向类型：`FORWARD` 表示直行连接；`LEFT` 和 `RIGHT` 分别表示左变道和右变道连接。

### 3. 构造函数

```cpp
TopoEdge::TopoEdge(const Edge& edge, const TopoNode* from_node,
                   const TopoNode* to_node)
    : pb_edge_(edge), from_node_(from_node), to_node_(to_node) {}
```

### 4. 属性函数

```cpp

const Edge& TopoEdge::PbEdge() const { return pb_edge_; }

double TopoEdge::Cost() const { return pb_edge_.cost(); }

const TopoNode* TopoEdge::FromNode() const { return from_node_; }

const TopoNode* TopoEdge::ToNode() const { return to_node_; }

const std::string& TopoEdge::FromLaneId() const {
  return pb_edge_.from_lane_id();
}

const std::string& TopoEdge::ToLaneId() const { return pb_edge_.to_lane_id(); }

TopoEdgeType TopoEdge::Type() const {
  if (pb_edge_.direction_type() == Edge::LEFT) {
    return TET_LEFT;
  }
  if (pb_edge_.direction_type() == Edge::RIGHT) {
    return TET_RIGHT;
  }
  return TET_FORWARD;
}
```



Reference:

