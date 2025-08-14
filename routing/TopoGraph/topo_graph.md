<!--
 * @Author: LOTEAT
 * @Date: 2025-08-13 10:25:02
-->

## TopoGraph详解

[知乎链接](https://zhuanlan.zhihu.com/p/1938986537074299709)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/TopoGraph/topo_graph.md)


### 1. Defination
```cpp
class TopoGraph {
 public:
  TopoGraph() = default;
  ~TopoGraph() = default;

  bool LoadGraph(const Graph& filename);

  const std::string& MapVersion() const;
  const std::string& MapDistrict() const;
  const TopoNode* GetNode(const std::string& id) const;
  void GetNodesByRoadId(
      const std::string& road_id,
      std::unordered_set<const TopoNode*>* const node_in_road) const;

 private:
  void Clear();
  bool LoadNodes(const Graph& graph);
  bool LoadEdges(const Graph& graph);

 private:
  std::string map_version_;
  std::string map_district_;
  std::vector<std::shared_ptr<TopoNode>> topo_nodes_;
  std::vector<std::shared_ptr<TopoEdge>> topo_edges_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::unordered_set<const TopoNode*>>
      road_node_map_;
};
```

`TopoGraph` 类将高精地图的车道网络抽象为一个完整的拓扑图结构，该类通过 `topo_nodes_` 和 `topo_edges_` 向量分别管理图中的所有节点和边对象。类中维护了 `node_index_map_` 哈希表用于根据车道ID快速定位节点，以及 `road_node_map_` 映射表用于根据道路ID批量获取相关节点集合。`TopoGraph` 还包含地图版本和区域信息。

### 2. Graph Message
```protobuf
message Graph {
  optional string hdmap_version = 1;
  optional string hdmap_district = 2;
  repeated Node node = 3;
  repeated Edge edge = 4;
}
```

`Graph` 消息是拓扑图的数据格式。`hdmap_version` 字段记录了高精地图的版本信息，确保拓扑图与底层地图数据的版本一致性。`hdmap_district` 字段标识地图所属的地理区域或行政区划。`repeated Node node` 字段包含了拓扑图中所有的节点定义，。`repeated Edge edge` 字段则定义了节点之间的所有连接关系，包括直行连接、变道连接等不同类型的边，并为每条边分配相应的通行代价。

### 3. LoadNodes
```cpp
bool TopoGraph::LoadNodes(const Graph& graph) {
  if (graph.node().empty()) {
    AERROR << "No nodes found in topology graph.";
    return false;
  }
  for (const auto& node : graph.node()) {
    node_index_map_[node.lane_id()] = static_cast<int>(topo_nodes_.size());
    std::shared_ptr<TopoNode> topo_node;
    topo_node.reset(new TopoNode(node));
    road_node_map_[node.road_id()].insert(topo_node.get());
    topo_nodes_.push_back(std::move(topo_node));
  }
  return true;
}
```

`LoadNodes` 函数是 `TopoGraph` 类中负责从序列化数据加载和构建拓扑节点，

对于每个节点，在将节点添加到 `topo_nodes_` 向量之前，先将其车道ID与当前向量大小的映射关系记录到 `node_index_map_` 中，这样就可以通过车道ID索引到对应的TopoNode。同时，这里还通过`road_node_map_`来维护road_id和TopoNode之间的关系。

### 4. LoadEdges
```cpp
bool TopoGraph::LoadEdges(const Graph& graph) {
  if (graph.edge().empty()) {
    AINFO << "0 edges found in topology graph, but it's fine";
    return true;
  }
  for (const auto& edge : graph.edge()) {
    const std::string& from_lane_id = edge.from_lane_id();
    const std::string& to_lane_id = edge.to_lane_id();
    if (node_index_map_.count(from_lane_id) != 1 ||
        node_index_map_.count(to_lane_id) != 1) {
      return false;
    }
    std::shared_ptr<TopoEdge> topo_edge;
    TopoNode* from_node = topo_nodes_[node_index_map_[from_lane_id]].get();
    TopoNode* to_node = topo_nodes_[node_index_map_[to_lane_id]].get();
    topo_edge.reset(new TopoEdge(edge, from_node, to_node));
    from_node->AddOutEdge(topo_edge.get());
    to_node->AddInEdge(topo_edge.get());
    topo_edges_.push_back(std::move(topo_edge));
  }
  return true;
}
```

`LoadEdges` 函数是 `TopoGraph` 类中负责构建拓扑图连接关系，它在节点加载完成后建立节点间的有向边连接。函数首先检查边数据的有效性，与节点加载不同的是，即使没有边数据也不会导致失败，因为某些特殊场景下的拓扑图可能只包含孤立节点。对于每条边，函数通过 `node_index_map_` 查找起始和目标节点，确保边引用的节点确实已经加载到图中，如果任一节点不存在则立即返回失败。在验证通过后，函数创建 `TopoEdge` 对象并建立双向关联关系：通过调用起始节点的 `AddOutEdge` 方法和目标节点的 `AddInEdge` 方法，将边信息同时注册到相关节点的边集合中。最后，函数将创建的边对象移动到 `topo_edges_` 向量中统一管理。

### 5. LoadGraph
```cpp
bool TopoGraph::LoadGraph(const Graph& graph) {
  Clear();

  map_version_ = graph.hdmap_version();
  map_district_ = graph.hdmap_district();

  if (!LoadNodes(graph)) {
    AERROR << "Failed to load nodes from topology graph.";
    return false;
  }
  if (!LoadEdges(graph)) {
    AERROR << "Failed to load edges from topology graph.";
    return false;
  }
  AINFO << "Load Topo data successful.";
  return true;
}
```

### 6. 属性函数
```cpp
const TopoNode* TopoGraph::GetNode(const std::string& id) const {
  const auto& iter = node_index_map_.find(id);
  if (iter == node_index_map_.end()) {
    return nullptr;
  }
  return topo_nodes_[iter->second].get();
}

void TopoGraph::GetNodesByRoadId(
    const std::string& road_id,
    std::unordered_set<const TopoNode*>* const node_in_road) const {
  const auto& iter = road_node_map_.find(road_id);
  if (iter != road_node_map_.end()) {
    node_in_road->insert(iter->second.begin(), iter->second.end());
  }
}
```


