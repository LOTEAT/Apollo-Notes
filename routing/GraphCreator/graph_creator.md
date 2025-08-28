<!--
 * @Author: LOTEAT
 * @Date: 2025-08-26 09:02:21
-->

## Graph Creator详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/GraphCreator/graph_creator.md)

### 1. Graph Creator
```cpp
class GraphCreator {
 public:
  GraphCreator(const std::string& base_map_file_path,
               const std::string& dump_topo_file_path,
               const RoutingConfig& routing_conf);

  ~GraphCreator() = default;

  bool Create();

 private:
  void InitForbiddenLanes();
  std::string GetEdgeID(const std::string& from_id, const std::string& to_id);

  void AddEdge(
      const Node& from_node,
      const ::google::protobuf::RepeatedPtrField<hdmap::Id>& to_node_vec,
      const Edge::DirectionType& type);

  static bool IsValidUTurn(const hdmap::Lane& lane, const double radius);

 private:
  std::string base_map_file_path_;
  std::string dump_topo_file_path_;
  hdmap::Map pbmap_;
  Graph graph_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::string> road_id_map_;
  std::unordered_set<std::string> showed_edge_id_set_;
  std::unordered_set<std::string> forbidden_lane_id_set_;

  const RoutingConfig& routing_conf_;
};
```

`GraphCreator` 类是 Apollo 中负责从高精度地图生成路径拓扑图的组件。该类的构造函数接收地图文件路径、输出拓扑文件路径和导航配置参数（前面提到的损失系数），通过 `Create()` 方法完成整个拓扑图的构建过程。类内部维护了多个数据结构：`pbmap_` 存储解析后的高精度地图数据，`graph_` 保存生成的拓扑图结构，`node_index_map_` 提供节点ID到索引的快速映射，`road_id_map_` 维护车道与道路的对应关系。此外，`showed_edge_id_set_` 用于去重避免重复添加边，`forbidden_lane_id_set_` 记录禁行车道信息，而 `routing_conf_` 则提供路径规划的配置参数。

### 2. Create
```cpp
bool GraphCreator::Create() {
  if (absl::EndsWith(base_map_file_path_, ".xml")) {
    if (!hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_,
                                                    &pbmap_)) {
      AERROR << "Failed to load base map file from " << base_map_file_path_;
      return false;
    }
  } else {
    if (!cyber::common::GetProtoFromFile(base_map_file_path_, &pbmap_)) {
      AERROR << "Failed to load base map file from " << base_map_file_path_;
      return false;
    }
  }

  AINFO << "Number of lanes: " << pbmap_.lane_size();

  graph_.set_hdmap_version(pbmap_.header().version());
  graph_.set_hdmap_district(pbmap_.header().district());

  node_index_map_.clear();
  road_id_map_.clear();
  showed_edge_id_set_.clear();

  for (const auto& road : pbmap_.road()) {
    for (const auto& section : road.section()) {
      for (const auto& lane_id : section.lane_id()) {
        road_id_map_[lane_id.id()] = road.id().id();
      }
    }
  }

  InitForbiddenLanes();
  const double min_turn_radius =
      VehicleConfigHelper::GetConfig().vehicle_param().min_turn_radius();

  for (const auto& lane : pbmap_.lane()) {
    const auto& lane_id = lane.id().id();
    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane id: " << lane_id
             << " because its type is NOT CITY_DRIVING.";
      continue;
    }
    if (lane.turn() == hdmap::Lane::U_TURN &&
        !IsValidUTurn(lane, min_turn_radius)) {
      ADEBUG << "The u-turn lane radius is too small for the vehicle to turn";
      continue;
    }
    AINFO << "Current lane id: " << lane_id;
    node_index_map_[lane_id] = graph_.node_size();
    const auto iter = road_id_map_.find(lane_id);
    if (iter != road_id_map_.end()) {
      node_creator::GetPbNode(lane, iter->second, routing_conf_,
                              graph_.add_node());
    } else {
      AWARN << "Failed to find road id of lane " << lane_id;
      node_creator::GetPbNode(lane, "", routing_conf_, graph_.add_node());
    }
  }

  for (const auto& lane : pbmap_.lane()) {
    const auto& lane_id = lane.id().id();
    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane id: " << lane_id
             << " because its type is NOT CITY_DRIVING.";
      continue;
    }
    const auto& from_node = graph_.node(node_index_map_[lane_id]);

    AddEdge(from_node, lane.successor_id(), Edge::FORWARD);
    if (lane.length() < FLAGS_min_length_for_lane_change) {
      continue;
    }
    if (lane.has_left_boundary() && IsAllowedToCross(lane.left_boundary())) {
      AddEdge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
    }

    if (lane.has_right_boundary() && IsAllowedToCross(lane.right_boundary())) {
      AddEdge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
    }
  }

  if (!absl::EndsWith(dump_topo_file_path_, ".bin") &&
      !absl::EndsWith(dump_topo_file_path_, ".txt")) {
    AERROR << "Failed to dump topo data into file, incorrect file type "
           << dump_topo_file_path_;
    return false;
  }
  auto type_pos = dump_topo_file_path_.find_last_of(".") + 1;
  std::string bin_file = dump_topo_file_path_.replace(type_pos, 3, "bin");
  std::string txt_file = dump_topo_file_path_.replace(type_pos, 3, "txt");
  if (!cyber::common::SetProtoToASCIIFile(graph_, txt_file)) {
    AERROR << "Failed to dump topo data into file " << txt_file;
    return false;
  }
  AINFO << "Txt file is dumped successfully. Path: " << txt_file;
  if (!cyber::common::SetProtoToBinaryFile(graph_, bin_file)) {
    AERROR << "Failed to dump topo data into file " << bin_file;
    return false;
  }
  AINFO << "Bin file is dumped successfully. Path: " << bin_file;
  return true;
}
```

`Create` 函数负责完成从高精度地图到路径拓扑图的完整转换过程。函数首先根据输入文件的扩展名判断地图数据格式，如果是 XML 格式则使用 OpendriveAdapter 进行加载，否则直接通过 Protocol Buffers 格式加载地图数据。加载完成后，函数会设置拓扑图的版本信息和区域信息，并清空各种映射表为后续处理做准备。

接下来函数遍历地图中的所有道路和路段信息，建立车道ID到道路ID的映射关系。然后调用 InitForbiddenLanes 方法初始化禁行车道集合，并获取车辆的最小转弯半径参数，这些信息用于过滤不适合当前车辆行驶的车道。

在第一个主要循环中，函数遍历所有车道并创建对应的拓扑节点。对于每个车道，函数首先检查其是否在禁行列表中，然后对于掉头车道还要验证其转弯半径是否满足车辆的物理约束。通过这些检查的车道会被添加到节点索引映射表中，并通过 node_creator::GetPbNode 函数创建实际的拓扑节点。`node_index_map_`用于维护lane的id和在`graph_`对应node的索引关系。

第二个主要循环负责创建拓扑图中的边连接关系。函数再次遍历所有有效车道，为每个车道添加前向边、左变道边和右变道边。前向边连接当前车道与其后继车道，表示车辆可以直行通过。对于变道边，函数会检查车道长度是否满足最小变道长度要求，以及边界是否允许跨越，只有满足条件的相邻车道之间才会建立变道连接。

最后，函数将构建完成的拓扑图数据分别保存为文本格式和二进制格式的文件。文本格式便于人工查看和调试，而二进制格式则用于运行时的快速加载。


### 3. AddEdge
```cpp
std::string GraphCreator::GetEdgeID(const std::string& from_id,
                                    const std::string& to_id) {
  return from_id + "->" + to_id;
}

void GraphCreator::AddEdge(const Node& from_node,
                           const RepeatedPtrField<Id>& to_node_vec,
                           const Edge::DirectionType& type) {
  for (const auto& to_id : to_node_vec) {
    if (forbidden_lane_id_set_.find(to_id.id()) !=
        forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane [id = " << to_id.id();
      continue;
    }
    const std::string edge_id = GetEdgeID(from_node.lane_id(), to_id.id());
    if (showed_edge_id_set_.count(edge_id) != 0) {
      continue;
    }
    showed_edge_id_set_.insert(edge_id);
    const auto& iter = node_index_map_.find(to_id.id());
    if (iter == node_index_map_.end()) {
      continue;
    }
    const auto& to_node = graph_.node(iter->second);
    edge_creator::GetPbEdge(from_node, to_node, type, routing_conf_,
                            graph_.add_edge());
  }
}
```

`AddEdge` 函数是拓扑图构建过程中负责创建边连接的方法，它接收源节点、目标节点列表和边的方向类型作为参数。函数首先遍历所有目标节点，对每个目标节点进行一系列有效性检查：跳过禁行车道、使用 `GetEdgeID` 生成边的唯一标识符并检查是否已经存在该边以避免重复添加、验证目标节点是否在节点索引映射表中存在。通过所有检查后，函数将边标识符加入到已显示边集合中进行去重管理，然后通过节点索引映射获取目标节点对象，最终调用 `edge_creator::GetPbEdge` 创建实际的拓扑边并添加到图中。

### 4. IsValidUTurn
```cpp
bool GraphCreator::IsValidUTurn(const hdmap::Lane& lane, const double radius) {
  if (lane.turn() != hdmap::Lane::U_TURN) {  // not a u-turn
    return false;
  }
  // approximate the radius from start point, middle point and end point.
  if (lane.central_curve().segment().empty() ||
      !lane.central_curve().segment(0).has_line_segment()) {
    return false;
  }
  std::vector<PointENU> points;
  for (const auto& segment : lane.central_curve().segment()) {
    points.insert(points.end(), segment.line_segment().point().begin(),
                  segment.line_segment().point().end());
  }
  if (points.empty()) {
    return false;
  }
  Vec2d p1{points.front().x(), points.front().y()};
  const auto& mid = points[points.size() / 2];
  Vec2d p2{mid.x(), mid.y()};
  Vec2d p3{points.back().x(), points.back().y()};
  Vec2d q1 = ((p1 + p2) / 2);                  // middle of p1---p2
  Vec2d q2 = (p2 - p1).rotate(M_PI / 2) + q1;  // perpendicular to p1-p2
  Vec2d q3 = ((p2 + p3) / 2);                  // middle of p2 -- p3
  Vec2d q4 = (p3 - p2).rotate(M_PI / 2) + q3;  // perpendicular to p2-p3
  const double s1 = CrossProd(q3, q1, q2);
  if (std::fabs(s1) < kMathEpsilon) {  // q3 is the circle center
    return q3.DistanceTo(p1) >= radius;
  }
  const double s2 = CrossProd(q4, q1, q2);
  if (std::fabs(s2) < kMathEpsilon) {  // q4 is the circle center
    return q4.DistanceTo(p1) >= radius;
  }
  if (std::fabs(s1 - s2) < kMathEpsilon) {  // parallel case, a wide u-turn
    return true;
  }
  Vec2d center = q3 + (q4 - q3) * s1 / (s1 - s2);
  return p1.DistanceTo(center) >= radius;
}
```

`IsValidUTurn` 函数用于验证掉头车道是否满足车辆的物理转弯约束条件。函数首先检查车道类型是否为掉头类型，然后从车道的中心线几何数据中提取起点、中点和终点坐标。通过这三个关键点，函数使用几何算法计算掉头路径的近似转弯半径：构建两条垂直平分线分别通过起点-中点和中点-终点的中垂点，通过叉积运算求解这两条直线的交点作为转弯圆心。函数处理了多种特殊情况，包括某个中垂点恰好是圆心的情况、两条中垂线平行表示宽阔掉头的情况，以及一般情况下通过线性插值计算圆心位置。最终函数比较计算得到的转弯半径与车辆最小转弯半径，确保车辆能够安全完成掉头动作。

### 5. InitForbiddenLanes
```cpp
void GraphCreator::InitForbiddenLanes() {
  for (const auto& lane : pbmap_.lane()) {
    if (lane.type() != hdmap::Lane::CITY_DRIVING) {
      forbidden_lane_id_set_.insert(lane.id().id());
    }
  }
}
```

`InitForbiddenLanes` 函数用于初始化禁行车道集合，确保拓扑图构建过程中只包含适合自动驾驶车辆行驶的车道。该函数遍历高精度地图中的所有车道，检查每个车道的类型属性，将所有非城市道路驾驶类型（CITY_DRIVING）的车道标记为禁行车道并添加到 `forbidden_lane_id_set_` 集合中。这些被过滤的车道类型可能包括人行道、自行车道、紧急车道、停车场内部道路等不适合普通车辆正常行驶的区域。