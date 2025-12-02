<!--
 * @Author: LOTEAT
 * @Date: 2025-08-04 16:02:57
-->

## Routing Messages详解

[知乎链接]()

[Github]()

本篇博客主要介绍一下apollo的routing模块中所使用的message。

### 1. LaneWaypoint

```protobuf
message LaneWaypoint {
  optional string id = 1;
  optional double s = 2;
  optional apollo.common.PointENU pose = 3;
  // When the developer selects a point on the dreamview route editing
  // the direction can be specified by dragging the mouse
  // dreamview calculates the heading based on this to support construct lane way point with heading
  optional double heading = 4;
}
```

`LaneWaypoint` 用于精确描述车辆在高精地图中的位置和朝向信息。该消息结构包含四个关键字段：`id` 字段标识具体的车道ID，将路径点与高精地图中的车道元素关联。`s` 字段表示沿车道方向的累计距离，用于在车道坐标系中定位具体位置。`pose` 字段提供了路径点在全局坐标系（ENU坐标系）中的位置信息。`heading` 字段记录了方向角度，`heading` 字段支持开发者在 Dreamview 界面中通过拖拽鼠标来指定路径方向，系统会根据这个交互操作计算出相应的朝向角度。

要注意的时候，再实际单独使用或验证导航模块的时候，id+s或者pose+heading都可以实现导航，并非要全部输入。只不过通常导航命令由external_command模块传递。除此之外，还需注意一点的是，apollo的导航是指全局导航，该模块会给出全局路径，这并不是一个具体可行驶的路径。真正的车辆行驶的局部路径由planning模块给出。

### 2. LaneSegment

```protobuf
message LaneSegment {
  optional string id = 1;
  optional double start_s = 2;
  optional double end_s = 3;
}
```

`LaneSegment` 是用于描述车道片段的数据结构，它定义了车道中的一个连续区间。该消息包含三个核心字段：`id` 字段标识所属的车道ID，将该片段与高精地图中的具体车道关联；`start_s` 和 `end_s` 字段分别表示该片段在车道坐标系中的起始和结束位置，通过这两个参数可以定义车道中的任意连续区间。这种设计允许系统将完整的车道分解为多个片段，便于进行路径规划和控制。

### 3. DeadEndRoutingType

```protobuf
enum DeadEndRoutingType {
  ROUTING_OTHER = 0;
  ROUTING_IN = 1;
  ROUTING_OUT = 2;
}
```

`DeadEndRoutingType` 是用于标识死胡同路段路径规划类型的枚举，它帮助识别和处理特殊的道路拓扑情况。该枚举包含三个值：`ROUTING_OTHER` 表示其他类型的路径规划，通常用作默认值或处理一般性的路径情况；`ROUTING_IN` 表示进入死胡同的路径规划，当车辆需要驶入没有出口的道路区域时使用；`ROUTING_OUT` 表示驶出死胡同的路径规划，用于车辆从死胡同区域返回到主要道路网络的情况。

### 4. Measurement

```protobuf
message Measurement {
  optional double distance = 1;
}
```

测量距离。

### 5. ChangeLaneType

```protobuf
enum ChangeLaneType {
  FORWARD = 0;
  LEFT = 1;
  RIGHT = 2;
};
```

`ChangeLaneType` 是用于标识车道变更方向的枚举类型，它为路径规划系统提供了明确的车道变更指令信息。该枚举包含三个基本的行驶方向：`FORWARD` 表示直行，即保持在当前车道继续前进，不进行车道变更；`LEFT` 表示向左变道，指示车辆需要从当前车道切换到左侧相邻车道；`RIGHT` 表示向右变道，指示车辆需要从当前车道切换到右侧相邻车道。

### 6. Passage

```protobuf
message Passage {
  repeated LaneSegment segment = 1;
  optional bool can_exit = 2;
  optional ChangeLaneType change_lane_type = 3 [default = FORWARD];
}
```

`Passage` 是路径规划中的通道概念，它将多个连续的车道片段组织成一个完整的行驶通道，是构建复杂路径的重要数据结构。该消息包含三个关键字段：`segment` 字段是一个 `LaneSegment` 数组，用于存储构成该通道的所有车道片段，这些片段按顺序连接形成一条连续的行驶路径；`can_exit` 字段是一个布尔值，表示车辆是否可以从当前通道退出到其他车道或路径，这对于车道变更和路径重规划非常重要；`change_lane_type` 字段指定了通过该通道时的车道变更类型，默认值为 `FORWARD`，表示直行通过。

### 7. RoadSegment

```protobuf
message RoadSegment {
  optional string id = 1;
  repeated Passage passage = 2;
}
```

`RoadSegment` 是路径规划中更高层次的道路片段概念，它将多个通道（`Passage`）组织在一起，形成一个完整的道路段落。该消息包含两个核心字段：`id` 字段唯一标识该道路片段，通常对应高精地图中的道路ID，用于建立与地图数据的关联；`passage` 字段是一个 `Passage` 数组，包含了该道路片段内的所有可行驶通道，这些通道可能代表不同的车道组合或行驶选择。

### 8. RoutingRequest

```protobuf
message RoutingRequest {
  optional apollo.common.Header header = 1;
  // at least two points. The first is start point, the end is final point.
  // The routing must go through each point in waypoint.
  repeated apollo.routing.LaneWaypoint waypoint = 2;
  repeated apollo.routing.LaneSegment blacklisted_lane = 3;
  repeated string blacklisted_road = 4;
  optional bool broadcast = 5 [default = true];
  optional apollo.routing.ParkingInfo parking_info = 6 [deprecated = true];
  // If the start pose is set as the first point of "way_point".
  optional bool is_start_pose_set = 7 [default = false];
}
```

`RoutingRequest` 是 Apollo 导航系统的核心输入消息，它封装了完整的路径导航请求信息，是连接上层应用和路径规划服务的重要接口。该消息包含多个关键字段：`header` 提供标准的消息头信息，包含时间戳和序列号等元数据；`waypoint` 是路径规划的核心，包含至少两个路径点（起点和终点），系统必须确保规划的路径经过所有指定的路径点；`blacklisted_lane` 和 `blacklisted_road` 分别用于指定禁止通行的车道和道路，为路径规划提供约束条件，这在处理道路施工、交通管制或车辆限行等场景时非常有用；`broadcast` 字段控制是否广播路径规划结果，默认为 true（我没有看到代码中对这一字段的使用）；`parking_info` 字段用于停车相关信息（已废弃）；`is_start_pose_set` 字段指示起始位置是否已在路径点中设定。

### 9. RoutingResponse

```protobuf
message RoutingResponse {
  optional apollo.common.Header header = 1;
  repeated apollo.routing.RoadSegment road = 2;
  optional apollo.routing.Measurement measurement = 3;
  optional RoutingRequest routing_request = 4;

  // the map version which is used to build road graph
  optional bytes map_version = 5;
  optional apollo.common.StatusPb status = 6;
}
```

`RoutingResponse` 是 Apollo 路径导航系统的输出消息，它包含了导航服务根据 `RoutingRequest` 计算得出的完整路径信息和相关元数据。该消息包含六个关键字段：`header` 提供响应消息的标准头信息，用于消息的追踪和同步；`road` 字段是核心输出，包含一个 `RoadSegment` 数组，描述了从起点到终点的完整路径，这些道路片段按顺序连接形成可行驶的全局路径；`measurement` 字段提供路径的测量信息，如总距离等统计数据；`routing_request` 字段回传原始的路径规划请求；`map_version` 字段记录了用于构建路径图的高精地图版本，确保路径规划结果与地图数据的一致性；`status` 字段提供路径规划的执行状态，包括成功、失败或警告等信息。



Reference:

