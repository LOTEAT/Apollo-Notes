<!--
 * @Author: LOTEAT
 * @Date: 2025-08-26 09:51:49
-->

## BlackListRangeGenerator详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/BlackListRangeGenerator/black_list_range_generator.md)

### 1. BlackListRangeGenerator
```cpp
class BlackListRangeGenerator {
 public:
  BlackListRangeGenerator() = default;
  ~BlackListRangeGenerator() = default;

  void GenerateBlackMapFromRequest(const routing::RoutingRequest& request,
                                   const TopoGraph* graph,
                                   TopoRangeManager* const range_manager) const;

  void AddBlackMapFromTerminal(const TopoNode* src_node,
                               const TopoNode* dest_node, double start_s,
                               double end_s,
                               TopoRangeManager* const range_manager) const;
};
```
`BlackListRangeGenerator` 类是 Apollo 路径规划系统中负责生成和管理黑名单区域的核心组件，用于标记在路径规划过程中需要避开的道路区段。该类提供了两个主要功能接口：`GenerateBlackMapFromRequest` 方法根据路径规划请求和拓扑图信息生成相应的黑名单区域，而 `AddBlackMapFromTerminal` 方法则用于在起点和终点节点附近添加特定的黑名单范围。


### 2. GenerateBlackMapFromRequest
```cpp
void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const routing::RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  AddBlackMapFromLane(request, graph, range_manager);
  AddBlackMapFromRoad(request, graph, range_manager);
  range_manager->SortAndMerge();
}
```

`GenerateBlackMapFromRequest` 函数是黑名单区域生成的方法，负责根据路径规划请求中的禁行信息生成完整的黑名单区域映射。该函数采用分层处理策略，首先调用 `AddBlackMapFromLane` 方法处理车道级别的禁行信息，将请求中指定的禁行车道及其对应的区间范围添加到范围管理器中；然后调用 `AddBlackMapFromRoad` 方法处理道路级别的禁行信息，将整条道路或道路的特定区段标记为不可通行。在完成所有黑名单区域的添加后，函数调用范围管理器的 `SortAndMerge` 方法对所有区间进行排序和合并操作，消除重叠区间并优化数据结构，确保后续路径搜索算法能够高效地查询和使用这些约束条件，从而生成避开所有禁行区域的安全可行路径。

### 3. AddBlackMapFromLane
```cpp
void AddBlackMapFromLane(const routing::RoutingRequest& request,
                         const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& lane : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(lane.id());
    if (node) {
      range_manager->Add(node, lane.start_s(), lane.end_s());
    }
  }
}
```

`AddBlackMapFromLane` 函数专门负责处理路径规划请求中车道级别的黑名单信息，将指定的禁行车道区段添加到范围管理器中。该函数遍历请求中的所有黑名单车道列表，对每个禁行车道，首先通过车道ID在拓扑图中查找对应的拓扑节点，如果找到有效的节点，则调用范围管理器的 `Add` 方法将该车道的禁行区间（由起始位置 `start_s` 和结束位置 `end_s` 定义）添加到黑名单中。

### 4. AddBlackMapFromRoad
```cpp
void AddBlackMapFromRoad(const routing::RoutingRequest& request,
                         const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& road_id : request.blacklisted_road()) {
    std::unordered_set<const TopoNode*> road_nodes_set;
    graph->GetNodesByRoadId(road_id, &road_nodes_set);
    for (const auto& node : road_nodes_set) {
      range_manager->Add(node, 0.0, node->Length());
    }
  }
}
```

`AddBlackMapFromRoad` 函数负责处理路径规划请求中道路级别的黑名单信息，将整条道路标记为禁行区域。该函数遍历请求中的所有黑名单道路ID列表，对每个禁行道路，首先通过 `GetNodesByRoadId` 方法在拓扑图中查找该道路下的所有拓扑节点（即该道路包含的所有车道节点），然后将这些节点的完整长度范围（从起点0.0到节点长度）全部添加到黑名单中。


