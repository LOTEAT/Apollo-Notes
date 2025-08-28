<!--
 * @Author: LOTEAT
 * @Date: 2025-08-26 10:19:08
-->

## TopoRangeManager详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/TopoRangeManager/topo_range_manager.md)

### 1. TopoRangeManager
```cpp
class TopoRangeManager {
 public:
  TopoRangeManager() = default;
  virtual ~TopoRangeManager() = default;

  const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>& RangeMap()
      const;
  const std::vector<NodeSRange>* Find(const TopoNode* node) const;
  void PrintDebugInfo() const;

  void Clear();
  void Add(const TopoNode* node, double start_s, double end_s);
  void SortAndMerge();

 private:
  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_map_;
};
```

`TopoRangeManager` 类是 Apollo 路径规划系统中负责管理拓扑节点区间信息的数据结构，用于维护每个拓扑节点上的禁行区段范围。该类内部使用哈希映射 `range_map_` 将拓扑节点指针映射到对应的区间范围列表，通过 `Add` 方法可以向指定节点添加新的区间范围，`SortAndMerge` 方法负责对区间进行排序和合并以优化存储和查询效率。`RangeMap` 返回完整的映射表，`Find` 方法快速查找特定节点的区间列表，`PrintDebugInfo` 用于调试输出，`Clear` 方法清空所有数据。

### 2. merge_block_range
```cpp
void merge_block_range(const TopoNode* topo_node,
                       const std::vector<NodeSRange>& origin_range,
                       std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range(origin_range);
  std::sort(sorted_origin_range.begin(), sorted_origin_range.end());
  size_t cur_index = 0;
  auto total_size = sorted_origin_range.size();
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
`merge_block_range` 函数是用于合并和处理拓扑节点上重叠区间范围的工具函数，它接收拓扑节点、原始区间列表和输出区间列表作为参数。函数首先对输入的区间列表进行排序，然后通过迭代方式逐个处理每个区间：对于当前区间，函数会尝试与后续重叠的区间进行合并，通过 `MergeRangeOverlap` 方法将连续重叠的区间合并为一个更大的区间。接下来函数会检查合并后的区间是否与拓扑节点的有效范围有交集，如果没有交集则跳过该区间。对于有交集的区间，函数会将其裁剪到拓扑节点的有效范围内，确保输出的区间不会超出节点的物理边界。最后生成一个禁行区间。

### 3. 其他函数
```cpp
const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
TopoRangeManager::RangeMap() const {
  return range_map_;
}
const std::vector<NodeSRange>* TopoRangeManager::Find(
    const TopoNode* node) const {
  auto iter = range_map_.find(node);
  if (iter == range_map_.end()) {
    return nullptr;
  } else {
    return &(iter->second);
  }
}

void TopoRangeManager::PrintDebugInfo() const {
  for (const auto& map : range_map_) {
    for (const auto& range : map.second) {
      AINFO << "black lane id: " << map.first->LaneId()
            << ", start s: " << range.StartS() << ", end s: " << range.EndS();
    }
  }
}

void TopoRangeManager::Clear() { range_map_.clear(); }

void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
  NodeSRange range(start_s, end_s);
  range_map_[node].push_back(range);
}

void TopoRangeManager::SortAndMerge() {
  for (auto& iter : range_map_) {
    std::vector<NodeSRange> merged_range_vec;
    merge_block_range(iter.first, iter.second, &merged_range_vec);
    iter.second.assign(merged_range_vec.begin(), merged_range_vec.end());
  }
}
```