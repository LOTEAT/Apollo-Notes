<!--
 * @Author: LOTEAT
 * @Date: 2025-08-06 09:59:26
-->
## Routing详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/Routing/routing.md)

本篇博客主要介绍一下apollo中的`Routing`类。这是`RoutingComponent`中负责根据request生成response的类。

### 1. Defination
```cpp
class Routing {
  // friend class RoutingTestBase;
 public:
  Routing();

  /**
   * @brief module name
   */
  std::string Name() const;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init();

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start();

  /**
   * @brief destructor
   */
  virtual ~Routing() = default;

  bool Process(
      const std::shared_ptr<routing::RoutingRequest> &routing_request,
      routing::RoutingResponse *const routing_response);

 private:
  std::vector<routing::RoutingRequest> FillLaneInfoIfMissing(
      const routing::RoutingRequest &routing_request);

 private:
  std::unique_ptr<Navigator> navigator_ptr_;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  const hdmap::HDMap *hdmap_ = nullptr;
};

```

`Routing` 类是 Apollo 路径规划系统的核心业务逻辑实现，它封装了从路径请求到路径响应的完整处理流程。

`navigator_ptr_` 是实际执行路径搜索算法的导航器对象，`hdmap_` 指针是高精地图模块的指针，为路径规划提供了详细的道路网络信息。`monitor_logger_buffer_` 提供了监控和日志功能。

`Process` 方法是整个类的核心接口，它根据 `RoutingRequest` 规划导航路径，并把结果写到 `RoutingResponse` 。

之前说到过，`RoutingRequest`无需把所有信息全部写入，`FillLaneInfoIfMissing` 就是去补全缺少精确的车道级信息，该方法能够根据坐标信息自动匹配到最近的合适车道，并补全必要的车道 ID 和位置参数。

### 2. Init
```cpp
apollo::common::Status Routing::Init() {
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));

  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  return apollo::common::Status::OK();
}
```

`Init` 函数是 `Routing` 类的初始化函数，函数首先通过 `apollo::hdmap::RoutingMapFile()` 获取路径拓扑图文件的路径，这个图是拓扑结构图，生成方式后面会讲到。接着函数使用该文件路径创建 `Navigator` 对象并赋值给 `navigator_ptr_`，随后，函数通过 `apollo::hdmap::HDMapUtil::BaseMapPtr()` 获取高精地图的指针并赋值给 `hdmap_`。

### 3. Start
```cpp
apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}
```

`Start` 函数是 `Routing` 类的验证函数，它在初始化完成后进行状态检查。函数首先通过调用 `navigator_ptr_->IsReady()` 检查导航器是否已经完全准备就绪。

### 4. FillLaneInfoIfMissing
```cpp
std::vector<routing::RoutingRequest> Routing::FillLaneInfoIfMissing(
    const routing::RoutingRequest& routing_request) {
  std::vector<routing::RoutingRequest> fixed_requests;
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;
  routing::RoutingRequest fixed_request(routing_request);
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    if (lane_waypoint.has_id()) {
      continue;
    }

    // fill lane info when missing
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    constexpr double kRadius = 0.3;
    for (int i = 0; i < 20; ++i) {
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      if (lanes.size() > 0) {
        break;
      }
    }
    if (lanes.empty()) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return fixed_requests;  // return empty vector
    }
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);
      if (j == 0) {
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }
    }
  }
  // first routing_request
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
  for (const auto& m : additional_lane_waypoint_map) {
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        routing::RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }

  for (const auto& fixed_request : fixed_requests) {
    ADEBUG << "Fixed routing request:" << fixed_request.DebugString();
  }
  return fixed_requests;
}
```

`FillLaneInfoIfMissing` 函数在路径点信息不完整时进行自动匹配。函数的核心思想是当路径点只包含地理坐标信息而缺少车道级精确信息时，通过高精地图的空间索引功能自动查找最近的合适车道，并将这些车道信息补全到路径请求中。

函数采用了渐进式搜索策略来提高匹配的成功率和准确性。初始搜索半径设置为 0.3 米，如果在该范围内没有找到合适的车道，函数会逐步扩大搜索半径，最多进行 20 次迭代，每次增加 0.3 米的搜索范围。一旦找到候选车道，函数会调用 `GetProjection` 方法计算路径点在车道坐标系中的精确位置，包括沿车道方向的 s 坐标和垂直于车道方向的 l 坐标。

函数最重要的特性是处理多车道匹配的情况：
```cpp
  for (const auto& m : additional_lane_waypoint_map) {
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        routing::RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }
```
`additional_lane_waypoint_map`的储存结果是key为waypoint index，value是其他的可选lane_waypoint。针对输入的路径中某些坐标点存在多个候选车道（即车道重叠），系统会在已有路径请求的基础上复制并替换对应位置的车道 ID，从而生成多条可选的 RoutingRequest 组合路径，以供后续模块选择最优路径使用。

### 5. Process
```cpp
bool Routing::Process(
    const std::shared_ptr<routing::RoutingRequest>& routing_request,
    routing::RoutingResponse* const routing_response) {
  if (nullptr == routing_request) {
    AWARN << "Routing request is empty!";
    return true;
  }
  CHECK_NOTNULL(routing_response);
  AINFO << "Get new routing request:" << routing_request->DebugString();

  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);
  double min_routing_length = std::numeric_limits<double>::max();
  for (const auto& fixed_request : fixed_requests) {
    routing::RoutingResponse routing_response_temp;
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
      const double routing_length =
          routing_response_temp.measurement().distance();
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
  }
  if (min_routing_length < std::numeric_limits<double>::max()) {
    monitor_logger_buffer_.INFO("Routing success!");
    return true;
  }

  AERROR << "Failed to search route with navigator.";
  monitor_logger_buffer_.WARN("Routing failed! " +
                              routing_response->status().msg());
  return false;
}
```

`Process` 函数是 `Routing` 类的处理接口，它接收路径规划请求并生成最优的导航响应。函数首先调用 `FillLaneInfoIfMissing` 方法处理输入请求中可能缺失的车道信息，生成一系列完整的候选路径请求。然后函数遍历这些候选请求，使用 `navigator_ptr_` 的 `SearchRoute` 方法为每个请求计算具体的导航路径，并通过比较路径长度选择最短的有效路径作为最终结果。