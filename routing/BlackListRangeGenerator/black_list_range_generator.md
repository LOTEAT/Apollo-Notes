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


### 5. AddBlackMapFromTerminal
```cpp
void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  double start_length = src_node->Length();
  double end_length = dest_node->Length();

  static constexpr double kEpsilon = 1e-2;
  const double start_s_adjusted =
      (start_s > start_length && start_s - start_length <= kEpsilon)
          ? start_length
          : start_s;
  const double end_s_adjusted =
      (end_s > end_length && end_s - end_length <= kEpsilon) ? end_length
                                                             : end_s;

  if (start_s_adjusted < 0.0 || start_s_adjusted > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }
  if (end_s_adjusted < 0.0 || end_s_adjusted > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  double start_cut_s = MoveSBackward(start_s_adjusted, 0.0);
  range_manager->Add(src_node, start_cut_s, start_cut_s);
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);

  double end_cut_s = MoveSForward(end_s_adjusted, end_length);
  range_manager->Add(dest_node, end_cut_s, end_cut_s);
  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  range_manager->SortAndMerge();
}
```

`AddBlackMapFromTerminal` 函数是专门用于处理起点和终点节点附近黑名单区域的方法，它接收源节点、目标节点、起始位置、结束位置和范围管理器作为参数。该函数首先获取源节点和目标节点的长度信息，然后通过数值处理来调整输入的起始和结束位置坐标。函数使用 `kEpsilon`（1e-2）来处理浮点数精度问题，当输入的坐标值略微超出节点长度但在误差范围内时，会自动调整到节点的边界位置。

在完成坐标调整后，函数会对调整后的坐标进行有效性验证，确保起始和结束位置都在合法的范围内（0到节点长度之间），如果发现非法坐标会记录错误日志并提前返回。接下来函数进入核心的黑名单处理阶段：对于源节点，函数调用 `MoveSBackward` 方法向后移动起始位置以创建一个安全的切割点，然后在该点添加黑名单标记，并通过 `AddBlackMapFromOutParallel` 方法处理从该节点向外延伸的并行车道的黑名单信息。

对于目标节点，函数采用类似但相反的处理策略：调用 `MoveSForward` 方法向前移动结束位置创建切割点，在该点添加黑名单标记，并通过 `AddBlackMapFromInParallel` 方法处理向该节点汇聚的并行车道的黑名单信息。这种设计防止路径规划在这些关键位置产生不合理的路径选择，这可能是为了边界控制，防止扰动和过冲。

最后，函数调用范围管理器的 `SortAndMerge` 方法对所有添加的黑名单区间进行整理和优化。

### 6. MoveSBackward
```cpp
double MoveSBackward(double s, double lower_bound) {
  if (s < lower_bound) {
    AERROR << "Illegal s: " << s << ", lower bound: " << lower_bound;
    return s;
  }
  if (s - S_GAP_FOR_BLACK > lower_bound) {
    return (s - S_GAP_FOR_BLACK);
  } else {
    return ((s + lower_bound) / 2.0);
  }
}
```

`MoveSBackward` 函数是用于向后调整位置坐标的工具函数，主要用于在黑名单区域生成过程中创建安全的缓冲间隔。该函数接收当前位置 `s` 和下边界 `lower_bound` 作为参数，首先检查输入位置是否合法（不能小于下边界），如果非法则记录错误并返回原值。对于合法的输入，如果当前位置减去预定义的黑名单间隔 `S_GAP_FOR_BLACK` 后仍然大于下边界，则直接向后移动该间隔距离；否则为了避免超出边界，函数会计算当前位置与下边界的中点作为调整后的位置。


### 7. AddBlackMapFromOutParallel & AddBlackMapFromInParallel
```cpp
void AddBlackMapFromOutParallel(const TopoNode* node, double cut_ratio,
                                TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetOutParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

void AddBlackMapFromInParallel(const TopoNode* node, double cut_ratio,
                               TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetInParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}
```

`AddBlackMapFromOutParallel` 和 `AddBlackMapFromInParallel` 函数是专门用于在并行车道上添加黑名单区域的方法，它们分别处理向外扩展和向内汇聚的并行车道黑名单标记。这两个函数接收基准节点、切割比例和范围管理器作为参数，首先调用相应的并行车道搜索函数（`GetOutParallelLane` 或 `GetInParallelLane`）来获取所有相关的并行车道节点集合，然后从集合中移除基准节点自身以避免重复处理。对于搜索到的每个并行车道节点，函数会根据传入的切割比例计算出对应的绝对位置坐标（切割比例乘以节点长度），并在该位置添加黑名单标记。这种基于比例的同步标记机制确保了在多车道场景下，当某个车道的特定位置需要被标记为禁行时，其所有并行车道的相对应位置也会被同步标记，从而保持黑名单区域在整个道路横截面上的一致性，避免了路径规划在并行车道间产生不合理的频繁变道行为。



### 8. GetOutParallelLane & GetInParallelLane
```cpp
void GetOutParallelLane(const TopoNode* node,
                        std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->OutToLeftOrRightEdge()) {
    const auto* to_node = edge->ToNode();
    if (node_set->count(to_node) == 0) {
      node_set->emplace(to_node);
      GetOutParallelLane(to_node, node_set);
    }
  }
}

void GetInParallelLane(const TopoNode* node,
                       std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->InFromLeftOrRightEdge()) {
    const auto* from_node = edge->FromNode();
    if (node_set->count(from_node) == 0) {
      node_set->emplace(from_node);
      GetInParallelLane(from_node, node_set);
    }
  }
}

```

`GetOutParallelLane` 和 `GetInParallelLane` 是一对用于收集并行车道节点的递归函数，它们分别负责查找从当前节点向外延伸和向内汇聚的所有并行车道。`GetOutParallelLane` 函数遍历当前节点的所有向左或向右的出边，通过这些变道边找到相邻的并行车道节点，然后递归地继续搜索这些节点的并行车道，形成一个完整的向外扩展的并行车道网络。相对应地，`GetInParallelLane` 函数则遍历当前节点的所有向左或向右的入边，通过这些变道边找到能够汇入当前节点的并行车道，同样采用递归方式构建向内汇聚的并行车道集合。