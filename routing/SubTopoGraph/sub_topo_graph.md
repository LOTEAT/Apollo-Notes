<!--
 * @Author: LOTEAT
 * @Date: 2025-08-13 15:25:07
-->
## SubTopoGraph详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/SubTopoGraph/sub_topo_graph.md)

### 1. Defination
```cpp
class SubTopoGraph {
 public:
  SubTopoGraph(const std::unordered_map<const TopoNode*,
                                        std::vector<NodeSRange>>& black_map);
  ~SubTopoGraph();

  // edge: A -> B         not sub edge
  // 1. A has no sub node, B has no sub node
  //      return origin edge A -> B
  // 2. A has no sub node, B has sub node
  //      return all edge A -> B'
  //    if B is black lane and no valid edge,
  //      return empty set
  // 3. A has sub node, B has sub node
  //      return empty set
  // 4. A has sub node, B has no sub node
  //      return empty set
  // edge: A -> B is sub edge
  // 1. return empty set
  void GetSubInEdgesIntoSubGraph(
      const TopoEdge* edge,
      std::unordered_set<const TopoEdge*>* const sub_edges) const;

  // edge: A -> B         not sub edge
  // 1. A has no sub node, B has no sub node
  //      return origin edge A -> B
  // 2. A has no sub node, B has sub node
  //      return all edge A -> B'
  //    if B is black lane and no valid edge,
  //      return empty set
  // 3. A has sub node, B has sub node
  //      return empty set
  // 4. A has sub node, B has no sub node
  //      return empty set
  // edge: A -> B is sub edge
  // 1. return empty set
  void GetSubOutEdgesIntoSubGraph(
      const TopoEdge* edge,
      std::unordered_set<const TopoEdge*>* const sub_edges) const;

  const TopoNode* GetSubNodeWithS(const TopoNode* topo_node, double s) const;

 private:
  void InitSubNodeByValidRange(const TopoNode* topo_node,
                               const std::vector<NodeSRange>& valid_range);
  void InitSubEdge(const TopoNode* topo_node);

  void InitInSubNodeSubEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);
  void InitOutSubNodeSubEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);

  bool GetSubNodes(const TopoNode* node,
                   std::unordered_set<TopoNode*>* const sub_nodes) const;

  void AddPotentialEdge(const TopoNode* topo_node);
  void AddPotentialInEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);
  void AddPotentialOutEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);

 private:
  std::vector<std::shared_ptr<TopoNode>> topo_nodes_;
  std::vector<std::shared_ptr<TopoEdge>> topo_edges_;
  std::unordered_map<const TopoNode*, std::vector<NodeWithRange>>
      sub_node_range_sorted_map_;
  std::unordered_map<const TopoNode*, std::unordered_set<TopoNode*>>
      sub_node_map_;
};
```
这个`SubTopoGraph`类的作用，其实就是在原始道路拓扑图（TopoGraph）的基础上，根据指定的范围，把节点（TopoNode）切分成子节点（sub node），并且重新建立对应的子边（sub edge），方便在部分道路或特定路段范围内进行路径规划或图计算。

### 2. IsCloseEnough
```cpp
bool IsCloseEnough(double s1, double s2) {
  return std::fabs(s1 - s2) < MIN_DIFF_LENGTH;
}
```

`IsCloseEnough` 用于判断两个坐标位置是否足够接近的辅助工具函数。

### 3. MergeBlockRange
```cpp
void MergeBlockRange(const TopoNode* topo_node,
                     const std::vector<NodeSRange>& origin_range,
                     std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range;
  sorted_origin_range.insert(sorted_origin_range.end(), origin_range.begin(),
                             origin_range.end());
  sort(sorted_origin_range.begin(), sorted_origin_range.end());
  int cur_index = 0;
  int total_size = static_cast<int>(sorted_origin_range.size());
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {
      ++cur_index;
    }
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));
    block_range->push_back(std::move(range));
  }
}
```

`MergeBlockRange` 用于处理禁行区域范围合并的函数。该函数首先对输入的原始范围集合进行排序，然后通过遍历算法将相邻或重叠的范围进行合并。在合并完成后，函数会对每个合并后的范围进行边界裁剪，确保最终的禁行范围不会超出指定拓扑节点的有效范围。

### 4. GetSortedValidRange
```cpp
void GetSortedValidRange(const TopoNode* topo_node,
                         const std::vector<NodeSRange>& origin_range,
                         std::vector<NodeSRange>* valid_range) {
  std::vector<NodeSRange> block_range;
  MergeBlockRange(topo_node, origin_range, &block_range);
  double start_s = topo_node->StartS();
  double end_s = topo_node->EndS();
  std::vector<double> all_value;
  all_value.push_back(start_s);
  for (const auto& range : block_range) {
    all_value.push_back(range.StartS());
    all_value.push_back(range.EndS());
  }
  all_value.push_back(end_s);
  for (size_t i = 0; i < all_value.size(); i += 2) {
    NodeSRange new_range(all_value[i], all_value[i + 1]);
    valid_range->push_back(std::move(new_range));
  }
}
```

`GetSortedValidRange` 用于从给定的拓扑节点中提取所有可通行的有效区间。该函数首先调用 `MergeBlockRange` 来处理和合并所有的禁行区域，然后构建一个包含所有关键边界点的数组，这些边界点包括节点的起始和结束位置以及所有禁行区域的边界。提取禁行区域之间的空隙部分就得到了可行区域。


### 5. IsReachable
```cpp
bool IsReachable(const TopoNode* from_node, const TopoNode* to_node) {
  double start_s = to_node->StartS() / to_node->Length() * from_node->Length();
  start_s = std::max(start_s, from_node->StartS());
  double end_s = to_node->EndS() / to_node->Length() * from_node->Length();
  end_s = std::min(end_s, from_node->EndS());
  return (end_s - start_s > MIN_POTENTIAL_LANE_CHANGE_LEN);
}
```

`IsReachable` 是一个用于判断两个拓扑节点之间是否存在有效连接。该函数通过比例换算的方式，将目标节点的范围映射到源节点的坐标系中，然后计算两个节点的重叠区域长度，最终通过比较重叠区域的长度与最小变道长度阈值来判断是否满足变道或连接。


### 6. SubTopoGraph
```cpp
SubTopoGraph::SubTopoGraph(
    const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
        black_map) {
  std::vector<NodeSRange> valid_range;
  for (const auto& map_iter : black_map) {
    valid_range.clear();
    GetSortedValidRange(map_iter.first, map_iter.second, &valid_range);
    InitSubNodeByValidRange(map_iter.first, valid_range);
  }

  for (const auto& map_iter : black_map) {
    InitSubEdge(map_iter.first);
  }

  for (const auto& map_iter : black_map) {
    AddPotentialEdge(map_iter.first);
  }
}
```

`SubTopoGraph` 构造函数接收一个包含原始拓扑节点和对应禁行区域范围的映射表作为输入参数。构造函数首先遍历所有节点并调用 `GetSortedValidRange` 计算每个节点的有效通行区域，然后基于这些有效区域通过 `InitSubNodeByValidRange` 创建相应的子节点。接下来通过 `InitSubEdge` 为每个原始节点建立子边连接关系，确保子节点之间的拓扑连接正确性。最后调用 `AddPotentialEdge` 添加潜在的变道边。


### 7. InitSubNodeByValidRange
```cpp
void SubTopoGraph::InitSubNodeByValidRange(
    const TopoNode* topo_node, const std::vector<NodeSRange>& valid_range) {
  // Attention: no matter topo node has valid_range or not,
  // create map value first;
  auto& sub_node_vec = sub_node_range_sorted_map_[topo_node];
  auto& sub_node_set = sub_node_map_[topo_node];

  std::vector<TopoNode*> sub_node_sorted_vec;
  for (const auto& range : valid_range) {
    if (range.Length() < MIN_INTERNAL_FOR_NODE) {
      continue;
    }
    std::shared_ptr<TopoNode> sub_topo_node_ptr;
    sub_topo_node_ptr.reset(new TopoNode(topo_node, range));
    sub_node_vec.emplace_back(sub_topo_node_ptr.get(), range);
    sub_node_set.insert(sub_topo_node_ptr.get());
    sub_node_sorted_vec.push_back(sub_topo_node_ptr.get());
    topo_nodes_.push_back(std::move(sub_topo_node_ptr));
  }

  for (size_t i = 1; i < sub_node_sorted_vec.size(); ++i) {
    auto* pre_node = sub_node_sorted_vec[i - 1];
    auto* next_node = sub_node_sorted_vec[i];
    if (IsCloseEnough(pre_node->EndS(), next_node->StartS())) {
      Edge edge;
      edge.set_from_lane_id(topo_node->LaneId());
      edge.set_to_lane_id(topo_node->LaneId());
      edge.set_direction_type(Edge::FORWARD);
      edge.set_cost(0.0);
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(new TopoEdge(edge, pre_node, next_node));
      pre_node->AddOutEdge(topo_edge_ptr.get());
      next_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}
```

`InitSubNodeByValidRange` 是负责根据有效范围创建子节点并建立它们之间的连接关系。该函数遍历所有有效范围，为每个长度满足最小要求的范围创建对应的子节点，并将这些子节点存储在排序向量和集合中以便后续查找。在创建完所有子节点后，函数会检查相邻子节点之间的距离，如果两个相邻子节点的结束位置和开始位置足够接近，则会创建一条前向边连接这两个子节点，这样可以确保在同一车道内的子节点能够形成连续的路径。


### 8. GetSubNodes
```cpp
bool SubTopoGraph::GetSubNodes(
    const TopoNode* node,
    std::unordered_set<TopoNode*>* const sub_nodes) const {
  const auto& iter = sub_node_map_.find(node);
  if (iter == sub_node_map_.end()) {
    return false;
  }
  sub_nodes->clear();
  sub_nodes->insert(iter->second.begin(), iter->second.end());
  return true;
}
```

`GetSubNodes` 是一个用于查询指定原始拓扑节点对应的所有子节点的函数。该函数通过在 `sub_node_map_` 中查找给定的原始节点来获取其关联的所有子节点集合。

### 9. InitSubEdge
```cpp
void SubTopoGraph::InitSubEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }

  for (auto* sub_node : sub_nodes) {
    InitInSubNodeSubEdge(sub_node, topo_node->InFromAllEdge());
    InitOutSubNodeSubEdge(sub_node, topo_node->OutToAllEdge());
  }
}
```
`InitSubEdge` 是负责为指定原始拓扑节点的所有子节点建立与外部节点连接关系的函数。该函数首先通过 `GetSubNodes` 获取原始节点对应的所有子节点，如果该节点没有子节点则直接返回。对于每个子节点，函数会分别调用 `InitInSubNodeSubEdge` 和 `InitOutSubNodeSubEdge` 来处理入边和出边的连接关系。

### 10. InitInSubNodeSubEdge
```cpp
void SubTopoGraph::InitInSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        if (!sub_from_node->IsOverlapEnough(sub_node, in_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else if (in_edge->FromNode()->IsOverlapEnough(sub_node, in_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}
```

`InitInSubNodeSubEdge` 是负责为指定子节点建立入边连接关系。该函数遍历原始节点的所有入边，对于每条入边，首先检查其源节点是否也有对应的子节点。如果源节点有子节点，函数会遍历所有这些子节点，并通过 `IsOverlapEnough` 方法判断它们与目标子节点之间是否存在足够的重叠区域来支持连接，满足条件的子节点之间会建立新的拓扑边连接。如果源节点没有子节点但原始节点与目标子节点之间存在足够重叠，则直接在原始源节点和目标子节点之间建立连接。

### 11. InitOutSubNodeSubEdge
```cpp
void SubTopoGraph::InitOutSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (!sub_node->IsOverlapEnough(sub_to_node, out_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else if (sub_node->IsOverlapEnough(out_edge->ToNode(), out_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}
```
`InitOutSubNodeSubEdge`和`InitInSubNodeSubEdge`函数处理逻辑一样。

### 12. GetSubInEdgesIntoSubGraph
```cpp
void SubTopoGraph::GetSubInEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(to_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  for (const auto* sub_node : sub_nodes) {
    for (const auto* in_edge : sub_node->InFromAllEdge()) {
      if (in_edge->FromNode() == from_node) {
        sub_edges->insert(in_edge);
      }
    }
  }
}
```

`GetSubInEdgesIntoSubGraph` 是一个用于获取子拓扑图中特定边对应的所有入边集合的查询函数。该函数首先检查给定边的源节点或目标节点是否已经是子节点，或者目标节点是否有对应的子节点集合，如果不满足这些条件则直接返回原始边，因为这个时候边已经是子图的边了，或者说是唯一的（目标节点唯一）。当目标节点存在子节点时，函数会遍历所有这些子节点，并检查每个子节点的入边集合，查找那些源节点与原始边源节点相同的边，将这些边添加到结果集合中。


### 13. GetSubOutEdgesIntoSubGraph
```cpp
void SubTopoGraph::GetSubOutEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(from_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  for (const auto* sub_node : sub_nodes) {
    for (const auto* out_edge : sub_node->OutToAllEdge()) {
      if (out_edge->ToNode() == to_node) {
        sub_edges->insert(out_edge);
      }
    }
  }
}
```
逻辑和`GetSubInEdgesIntoSubGraph`一样。

### 14. GetSubNodeWithS
```cpp
const TopoNode* SubTopoGraph::GetSubNodeWithS(const TopoNode* topo_node,
                                              double s) const {
  const auto& map_iter = sub_node_range_sorted_map_.find(topo_node);
  if (map_iter == sub_node_range_sorted_map_.end()) {
    return topo_node;
  }
  const auto& sorted_vec = map_iter->second;
  // sorted vec can't be empty!
  int index = BinarySearchForStartS(sorted_vec, s);
  if (index < 0) {
    return nullptr;
  }
  return sorted_vec[index].GetTopoNode();
}
```

`GetSubNodeWithS` 是一个根据给定的纵向坐标位置查找对应子节点的精确定位函数。该函数首先在排序映射表中查找指定的原始拓扑节点，如果该节点没有对应的子节点则直接返回原始节点本身。当找到子节点集合后，函数使用二分搜索算法 `BinarySearchForStartS` 在已排序的子节点向量中快速定位包含给定坐标 s 的子节点。如果搜索成功找到有效的索引，则返回对应的子节点；如果搜索失败（索引为负），则返回空指针表示该位置没有有效的子节点。


### 15. AddPotentialEdge
```cpp
void SubTopoGraph::AddPotentialEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }
  for (auto* sub_node : sub_nodes) {
    AddPotentialInEdge(sub_node, topo_node->InFromLeftOrRightEdge());
    AddPotentialOutEdge(sub_node, topo_node->OutToLeftOrRightEdge());
  }
}
```

`AddPotentialEdge` 是负责为子节点添加潜在变道边连接的函数。该函数首先通过 `GetSubNodes` 获取指定原始拓扑节点对应的所有子节点，如果该节点没有子节点则直接返回。对于每个子节点，函数会分别调用 `AddPotentialInEdge` 和 `AddPotentialOutEdge` 来处理来自左右车道的入边和出边连接关系。

### 16. AddPotentialInEdge
```cpp
void SubTopoGraph::AddPotentialInEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        if (sub_node->GetInEdgeFrom(sub_from_node) != nullptr) {
          continue;
        }
        if (!IsReachable(sub_from_node, sub_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetInEdgeFrom(in_edge->FromNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}
```
`AddPotentialInEdge` 是负责为指定子节点添加潜在入边连接的函数，主要处理来自相邻车道的变道连接。该函数遍历原始节点的所有左右车道入边，对于每条潜在入边，首先检查其源节点是否有对应的子节点。如果源节点有子节点，函数会遍历所有这些子节点，并进行严格的连接条件检查：首先验证目标子节点是否已经存在来自该源子节点的入边以避免重复连接，然后通过 `IsReachable` 函数判断两个子节点之间是否满足变道的几何和距离要求。如果源节点没有子节点，则直接检查目标子节点是否已存在来自该原始节点的入边，满足条件时建立连接。

### 17. AddPotentialOutEdge
```cpp
void SubTopoGraph::AddPotentialOutEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (sub_node->GetOutEdgeTo(sub_to_node) != nullptr) {
          continue;
        }
        if (!IsReachable(sub_node, sub_to_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetOutEdgeTo(out_edge->ToNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}
```
思路和`AddPotentialInEdge`一致。