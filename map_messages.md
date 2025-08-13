## Map Messages详解

[知乎链接]()

[Github]()

Map Messages是Apollo自动驾驶系统中地图模块的核心组件，承担着地图数据结构定义和信息传递的重要职责。作为整个地图系统的基础，Map Messages定义了Apollo系统中各种地图元素的数据格式。

Apollo所有的message军事基于Protocol Buffers 2（protobuf2）来定义数据结构。Protocol Buffers是Google开发的一种语言无关、平台无关的序列化数据结构协议，具有高效、紧凑、可扩展等特点。如果你还不熟悉protobuf，建议先学习protobuf的基础知识，这将帮助你更好地理解Map Messages的定义和使用方式。

### 1. Map Id

```protobuf
syntax = "proto2";

package apollo.hdmap;

// Global unique ids for all objects (include lanes, junctions, overlaps, etc).
// map id for all objects
message Id {
  optional string id = 1;
}
```

`Id` 消息是Apollo地图系统中最基础的数据结构，为所有地图对象（车道、路口、重叠区域等）提供全局唯一标识符。

-   **id**: 字符串类型的唯一标识符，用于标识和引用地图中的各种对象

这个简单的结构是整个地图系统的基础，所有复杂的地图对象都会使用这个Id来建立身份标识。

### 2. Geometry

```protobuf
syntax = "proto2";

package apollo.common;

// A point in the map reference frame. The map defines an origin, whose
// coordinate is (0, 0, 0).
// Most modules, including localization, perception, and prediction, generate
// results based on the map reference frame.
// Currently, the map uses Universal Transverse Mercator (UTM) projection. See
// the link below for the definition of map origin.
//   https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
// The z field of PointENU can be omitted. If so, it is a 2D location and we do
// not care its height.
message PointENU {
  optional double x = 1 [default = nan];  // East from the origin, in meters.
  optional double y = 2 [default = nan];  // North from the origin, in meters.
  optional double z = 3 [default = 0.0];  // Up from the WGS-84 ellipsoid, in
                                          // meters.
}

// A point in the global reference frame. Similar to PointENU, PointLLH allows
// omitting the height field for representing a 2D location.
message PointLLH {
  // Longitude in degrees, ranging from -180 to 180.
  optional double lon = 1 [default = nan];
  // Latitude in degrees, ranging from -90 to 90.
  optional double lat = 2 [default = nan];
  // WGS-84 ellipsoid height in meters.
  optional double height = 3 [default = 0.0];
}

// A general 2D point. Its meaning and units depend on context, and must be
// explained in comments.
message Point2D {
  optional double x = 1 [default = nan];
  optional double y = 2 [default = nan];
}

// A general 3D point. Its meaning and units depend on context, and must be
// explained in comments.
message Point3D {
  optional double x = 1 [default = nan];
  optional double y = 2 [default = nan];
  optional double z = 3 [default = nan];
}

// A unit quaternion that represents a spatial rotation. See the link below for
// details.
//   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
// The scalar part qw can be omitted. In this case, qw should be calculated by
//   qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
message Quaternion {
  optional double qx = 1 [default = nan];
  optional double qy = 2 [default = nan];
  optional double qz = 3 [default = nan];
  optional double qw = 4 [default = nan];
}

// A general polygon, points are counter clockwise
message Polygon {
  repeated Point3D point = 1;
}

```

Geometry相关的数据结构定义了Apollo系统中使用的基础几何类型，虽然不是地图专用消息，但在地图数据中广泛使用。

#### 坐标点类型：

-   **PointENU**: 地图参考系坐标点，使用UTM投影，x(东)、y(北)、z(高度)
-   **PointLLH**: 全球坐标系点，使用经度(lon)、纬度(lat)、高度(height)
-   **Point2D/Point3D**: 通用2D/3D坐标点，具体含义根据上下文确定

#### 其他几何类型：

-   **Quaternion**: 四元数，表示空间旋转
-   **Polygon**: 多边形，由多个3D点组成（逆时针顺序）

这些基础几何类型是构建复杂地图元素（如车道线、路口等）的基石。

### 3. Map Geometry

```protobuf
syntax = "proto2";

import "modules/common_msgs/basic_msgs/geometry.proto";

package apollo.hdmap;

// Polygon, not necessary convex.
// make a polygon by using many PointENU
message Polygon {
  repeated apollo.common.PointENU point = 1;
}

// Straight line segment.
// line segment
message LineSegment {
  repeated apollo.common.PointENU point = 1;
}

// Generalization of a line.
message CurveSegment {
  // oneof is like union
  // but there is only one type
  // why use oneof here
  oneof curve_type {
    LineSegment line_segment = 1;
  }
  // st coordinate in opendrive system
  optional double s = 6;  // start position (s-coordinate)
  optional apollo.common.PointENU start_position = 7;
  optional double heading = 8;  // start orientation
  optional double length = 9;
}

// An object similar to a line but that need not be straight.
message Curve {
  repeated CurveSegment segment = 1;
}

```

Map Geometry定义了地图专用的几何数据结构，基于通用Geometry类型构建地图元素。

**基础几何结构：**

-   **Polygon**: 多边形，由多个PointENU点组成
-   **LineSegment**: 直线段，由一系列PointENU点构成
-   **CurveSegment**: 曲线段，目前只支持直线段类型
    -   使用oneof预留扩展性
    -   包含起始位置、方向、长度等参数

-   **Curve**: 复合曲线，由多个CurveSegment组成

这些结构是构建车道、道路边界、停车线等地图元素的基础几何组件。

### 3. Clear Area

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// A clear area means in which stopping car is prohibited

message ClearArea {
  optional Id id = 1;
  repeated Id overlap_id = 2;
  // the shape of clear area
  optional Polygon polygon = 3;
}
```

Clear Area（禁停区）是Apollo高精地图中的一个重要元素，定义了车辆禁止停车的区域。这些区域在自动驾驶系统中至关重要，因为它们确保车辆不会在可能导致交通阻塞或安全隐患的地方停车。

**数据结构说明：**

1.  **id** (Id类型):

    -   每个Clear Area的唯一标识符
    -   用于系统内对该禁停区的引用

1.  **overlap_id** (多个Id类型):

    -   与该禁停区域重叠的其他地图元素的ID列表
    -   通常包含与该区域相交的车道(Lane)、路口(Junction)等元素

1.  **polygon** (Polygon类型):

    -   定义禁停区的几何形状和边界
    -   由多个PointENU点组成，形成一个封闭多边形
    -   系统通过判断车辆是否位于这个多边形内来确定是否处于禁停区

### 4. Crosswalk

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// Crosswalk is a place designated for pedestrians to cross a road.
message Crosswalk {
  // crosswalk id
  optional Id id = 1;
  // crosswalk
  optional Polygon polygon = 2;
  // overlap object id
  repeated Id overlap_id = 3;
}
```

Crosswalk（人行横道）是用于描述供行人安全穿越道路的区域。自动驾驶系统通过识别和理解人行横道的位置和范围，可以在行驶过程中做出减速、停车等决策，保障行人安全。

**数据结构说明：**

-   **id**（Id类型）  
    每个人行横道的唯一标识符，用于系统内部引用和管理。

-   **polygon**（Polygon类型）  
    定义人行横道的几何形状和边界。由多个点（Point3D）组成，形成一个封闭多边形，精确描述人行横道在地图上的实际范围。

-   **overlap_id**（多个Id类型）  
    与该人行横道重叠的其他地图元素的ID列表。常见的重叠对象包括车道（Lane）、路口（Junction）等，用于描述人行横道与其他交通元素的空间关系。

### 5. Junction

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// A junction is the junction at-grade of two or more roads crossing.
message Junction {
  // junction id
  optional Id id = 1;
  // shape
  optional Polygon polygon = 2;

  repeated Id overlap_id = 3;
  enum Type {
    UNKNOWN = 0;
    IN_ROAD = 1;
    CROSS_ROAD = 2;
    FORK_ROAD = 3;
    MAIN_SIDE = 4;
    DEAD_END = 5;
  };
  optional Type type = 4;
}
```

Junction（路口）是Apollo高精地图中的重要元素之一，用于描述两条或多条道路平面交汇的区域。自动驾驶系统通过识别和理解路口的位置、范围和类型。

**数据结构说明：**

-   **id**（Id类型）  
    每个路口的唯一标识符，用于系统内部引用和管理。

-   **polygon**（Polygon类型）  
    定义路口的几何形状和边界。由多个点（Point3D）组成，形成一个封闭多边形，精确描述路口在地图上的实际范围。

-   **overlap_id**（多个Id类型）  
    与该路口重叠的其他地图元素的ID列表。常见的重叠对象包括车道（Lane）、人行横道（Crosswalk）等，用于描述路口与其他交通元素的空间关系。

-   **type**（Type枚举）  
    路口类型，常见类型包括：

    -   UNKNOWN：未知类型
    -   IN_ROAD：道路内部交汇
    -   CROSS_ROAD：十字路口
    -   FORK_ROAD：岔路口
    -   MAIN_SIDE：主辅路交汇
    -   DEAD_END：死胡同

### 6. Parking Space

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// ParkingSpace is a place designated to park a car.
message ParkingSpace {
  optional Id id = 1;

  optional Polygon polygon = 2;

  repeated Id overlap_id = 3;
  // parking heading
  optional double heading = 4;
}

// ParkingLot is a place for parking cars.
message ParkingLot {
  optional Id id = 1;

  optional Polygon polygon = 2;

  repeated Id overlap_id = 3;
}

```

ParkingSpace（停车位）用于描述地图中单个可供车辆停放的具体位置。

**数据结构说明：**

-   **id**（Id类型）  
    每个停车位的唯一标识符，用于系统内部引用和管理。

-   **polygon**（Polygon类型）  
    定义停车位的几何形状和边界。由多个点（Point3D）组成，形成一个封闭多边形，精确描述停车位在地图上的实际范围。

-   **overlap_id**（多个Id类型）  
    与该停车位重叠的其他地图元素的ID列表。常见的重叠对象包括车道（Lane）、道路（Road）等，用于描述停车位与其他交通元素的空间关系。

-   **heading**（double类型）  
    停车位的朝向角度，单位为弧度，表示车辆在该停车位内的标准停放方向。

### 7. RSU

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_id.proto";
// Roadside Unit information
message RSU {
  optional Id id = 1;
  optional Id junction_id = 2;
  repeated Id overlap_id = 3;
};

```

RSU（Roadside Unit）用于描述道路基础设施中的路侧通信设备信息。

**数据结构说明：**

-   **id**（Id类型）：RSU的唯一标识符。
-   **junction_id**（Id类型）：该RSU关联的路口ID。
-   **overlap_id**（多个Id类型）：与该RSU重叠的其他地图元素的ID列表。

### 8. Signal

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/basic_msgs/geometry.proto";
import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

message Subsignal {
  // signal type
  enum Type {
    UNKNOWN = 1;
    CIRCLE = 2;
    ARROW_LEFT = 3;
    ARROW_FORWARD = 4;
    ARROW_RIGHT = 5;
    ARROW_LEFT_AND_FORWARD = 6;
    ARROW_RIGHT_AND_FORWARD = 7;
    ARROW_U_TURN = 8;
  };
  // id
  optional Id id = 1;
  optional Type type = 2;

  // Location of the center of the bulb. now no data support.
  optional apollo.common.PointENU location = 3;
}


message SignInfo {
  enum Type {
    None = 0;
    NO_RIGHT_TURN_ON_RED = 1;
  };

  optional Type type = 1;
}

message Signal {
  enum Type {
    UNKNOWN = 1;
    // horizontal and vertical directions and double lights?
    MIX_2_HORIZONTAL = 2;
    MIX_2_VERTICAL = 3;
    // horizontal and vertical directions and three lights?
    MIX_3_HORIZONTAL = 4;
    MIX_3_VERTICAL = 5;
    // single light
    SINGLE = 6;
  };

  optional Id id = 1;
  optional Polygon boundary = 2;
  repeated Subsignal subsignal = 3;
  // TODO: add orientation. now no data support.
  repeated Id overlap_id = 4;
  optional Type type = 5;
  // stop line
  repeated Curve stop_line = 6;

  repeated SignInfo sign_info = 7;
}

```

Signal（信号灯）用于描述道路上的交通信号灯及其属性。

**数据结构说明：**

-   **id**（Id类型）：信号灯的唯一标识符。
-   **boundary**（Polygon类型）：信号灯的边界区域。
-   **subsignal**（Subsignal类型，多个）：信号灯的子灯组信息，包含类型、位置等。
-   **overlap_id**（多个Id类型）：与该信号灯重叠的其他地图元素的ID列表。
-   **type**（Type枚举）：信号灯类型，如单灯、双灯、三灯等。
-   **stop_line**（Curve类型，多个）：停止线的几何信息。
-   **sign_info**（SignInfo类型，多个）：交通标志信息。

### 9. Speed Bump

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

message SpeedBump {
  // speed bump id
  optional Id id = 1;
  // overlap object id
  repeated Id overlap_id = 2;
  // position
  repeated Curve position = 3;
}
```

SpeedBump（减速带）用于描述道路上的减速带信息。

**数据结构说明：**

-   **id**（Id类型）：减速带的唯一标识符。
-   **overlap_id**（多个Id类型）：与该减速带重叠的其他地图元素的ID列表。
-   **position**（Curve类型，多个）：减速带的位置和几何形状。

### 10. Speed Control

```protobuf
syntax = "proto2";

import "modules/common_msgs/map_msgs/map_geometry.proto";

package apollo.hdmap;

// This proto defines the format of an auxiliary file that helps to
// define the speed limit on certain area of road.
// Apollo can use this file to quickly fix speed problems on maps,
// instead of waiting for updating map data.
// auxiliary file
message SpeedControl {
  optional string name = 1;
  optional apollo.hdmap.Polygon polygon = 2;
  optional double speed_limit = 3;
}

message SpeedControls {
  repeated SpeedControl speed_control = 1;
}
```

SpeedControl（限速区域）用于描述地图中某一特定区域的限速信息。

**数据结构说明：**

-   **name**（string类型）：限速区域的名称。
-   **polygon**（Polygon类型）：定义限速区域的几何形状和边界，由多个点（Point3D）组成，形成一个封闭多边形。
-   **speed_limit**（double类型）：该区域的限速值，单位为米/秒（m/s）。

SpeedControls为SpeedControl的集合。

### 11. Stop Sign

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// A stop sign is a traffic sign to notify drivers that they must stop before
// proceeding.
message StopSign {
  // stop sign id
  optional Id id = 1;
  // stop line
  repeated Curve stop_line = 2;
  // overlap object id
  repeated Id overlap_id = 3;
  // stop type
  enum StopType {
    UNKNOWN = 0;
    ONE_WAY = 1;
    TWO_WAY = 2;
    THREE_WAY = 3;
    FOUR_WAY = 4;
    ALL_WAY = 5;
  };
  optional StopType type = 4;
}

```

StopSign（停车标志）用于描述道路上的停车标志及其相关信息。

**数据结构说明：**

-   **id**（Id类型）：停车标志的唯一标识符。
-   **stop_line**（Curve类型，多个）：停车标志对应的停止线的几何信息。
-   **overlap_id**（多个Id类型）：与该停车标志重叠的其他地图元素的ID列表。
-   **type**（StopType枚举）：停车标志的类型，包括：
    -   UNKNOWN：未知类型
    -   ONE_WAY：单向停车标志
    -   TWO_WAY：双向停车标志
    -   THREE_WAY：三向停车标志
    -   FOUR_WAY：四向停车标志
    -   ALL_WAY：全向停车标志

### 12. Yield Sign

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// A yield indicates that each driver must prepare to stop if necessary to let a
// driver on another approach proceed.
// A driver who stops or slows down to let another vehicle through has yielded
// the right of way to that vehicle.
// I do not understand
message YieldSign {
  optional Id id = 1;

  repeated Curve stop_line = 2;

  repeated Id overlap_id = 3;
}
```

YieldSign（让行标志）用于描述道路上的让行标志及其相关信息。

**数据结构说明：**

-   **id**（Id类型）：让行标志的唯一标识符。
-   **stop_line**（Curve类型，多个）：让行标志对应的停止线的几何信息。
-   **overlap_id**（多个Id类型）：与该让行标志重叠的其他地图元素的ID列表。

### 13. Lane

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// lane boundary type
message LaneBoundaryType {
  enum Type {
    UNKNOWN = 0;
    DOTTED_YELLOW = 1;
    DOTTED_WHITE = 2;
    SOLID_YELLOW = 3;
    SOLID_WHITE = 4;
    DOUBLE_YELLOW = 5;
    CURB = 6;
  };
  // Offset relative to the starting point of boundary
  // st coordinate
  optional double s = 1;
  // support multiple types
  repeated Type types = 2;
}

message LaneBoundary {
  optional Curve curve = 1;

  optional double length = 2;
  // indicate whether the lane boundary exists in real world
  optional bool virtual = 3;
  // in ascending order of s
  repeated LaneBoundaryType boundary_type = 4;
}

// Association between central point to closest boundary.
// sample point?
message LaneSampleAssociation {
  optional double s = 1;
  optional double width = 2;
}

// A lane is part of a roadway, that is designated for use by a single line of
// vehicles.
// Most public roads (include highways) have more than two lanes.
message Lane {
  optional Id id = 1;

  // Central lane as reference trajectory, not necessary to be the geometry
  // central.
  optional Curve central_curve = 2;

  // Lane boundary curve.
  optional LaneBoundary left_boundary = 3;
  optional LaneBoundary right_boundary = 4;

  // in meters.
  optional double length = 5;

  // Speed limit of the lane, in meters per second.
  optional double speed_limit = 6;

  repeated Id overlap_id = 7;

  // All lanes can be driving into (or from).
  repeated Id predecessor_id = 8;
  repeated Id successor_id = 9;

  // Neighbor lanes on the same direction.
  repeated Id left_neighbor_forward_lane_id = 10;
  repeated Id right_neighbor_forward_lane_id = 11;

  enum LaneType {
    NONE = 1;
    CITY_DRIVING = 2;
    BIKING = 3;
    SIDEWALK = 4;
    PARKING = 5;
    SHOULDER = 6;
  };
  optional LaneType type = 12;

  enum LaneTurn {
    NO_TURN = 1;
    LEFT_TURN = 2;
    RIGHT_TURN = 3;
    U_TURN = 4;
  };
  optional LaneTurn turn = 13;
  // left reverse direction lane
  repeated Id left_neighbor_reverse_lane_id = 14;
  // right reverse direction lane
  repeated Id right_neighbor_reverse_lane_id = 15;

  optional Id junction_id = 16;

  // Association between central point to closest boundary.
  repeated LaneSampleAssociation left_sample = 17;
  repeated LaneSampleAssociation right_sample = 18;

  enum LaneDirection {
    FORWARD = 1;
    BACKWARD = 2;
    BIDIRECTION = 3;
  }
  optional LaneDirection direction = 19;

  // Association between central point to closest road boundary.
  repeated LaneSampleAssociation left_road_sample = 20;
  repeated LaneSampleAssociation right_road_sample = 21;
  // reverse lane id
  repeated Id self_reverse_lane_id = 22;
}

```

Lane（车道）用于描述道路上的单条车道及其详细属性，是高精地图中最基础、最核心的元素之一。

**数据结构说明：**

-   **id**（Id类型）：车道的唯一标识符。
-   **central_curve**（Curve类型）：车道中心线的几何信息，作为车辆行驶的参考轨迹。
-   **left_boundary/right_boundary**（LaneBoundary类型）：车道的左/右边界信息。
-   **length**（double类型）：车道长度，单位为米。
-   **speed_limit**（double类型）：车道限速，单位为米/秒（m/s）。
-   **overlap_id**（多个Id类型）：与该车道重叠的其他地图元素的ID列表。
-   **predecessor_id/successor_id**（多个Id类型）：前驱/后继车道ID。
-   **left_neighbor_forward_lane_id/right_neighbor_forward_lane_id**（多个Id类型）：同向左/右侧相邻车道ID。
-   **type**（LaneType枚举）：车道类型，如城市道路、非机动车道、人行道、停车道、路肩等。
-   **turn**（LaneTurn枚举）：车道转向类型，如直行、左转、右转、掉头等。
-   **left_neighbor_reverse_lane_id/right_neighbor_reverse_lane_id**（多个Id类型）：逆向左/右侧相邻车道ID。
-   **junction_id**（Id类型）：所属路口ID。
-   **left_sample/right_sample**（LaneSampleAssociation类型，多个）：中心线到左右边界的采样关联。
-   **direction**（LaneDirection枚举）：车道行驶方向，如正向、反向、双向。
-   **left_road_sample/right_road_sample**（LaneSampleAssociation类型，多个）：中心线到道路边界的采样关联。
-   **self_reverse_lane_id**（多个Id类型）：自身反向车道ID。

### 14. Road

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

// boundary edge
message BoundaryEdge {
  optional Curve curve = 1;
  enum Type {
    UNKNOWN = 0;
    NORMAL = 1;
    LEFT_BOUNDARY = 2;
    RIGHT_BOUNDARY = 3;
  };
  optional Type type = 2;
}

message BoundaryPolygon {
  repeated BoundaryEdge edge = 1;
}

// boundary with holes
message RoadBoundary {
  optional BoundaryPolygon outer_polygon = 1;
  // if boundary without hole, hole is null
  repeated BoundaryPolygon hole = 2;
}

message RoadROIBoundary {
  optional Id id = 1;
  repeated RoadBoundary road_boundaries = 2;
}

// road section defines a road cross-section, At least one section must be
// defined in order to
// use a road, If multiple road sections are defined, they must be listed in
// order along the road
message RoadSection {
  optional Id id = 1;
  // lanes contained in this section
  repeated Id lane_id = 2;
  // boundary of section
  optional RoadBoundary boundary = 3;
}

// The road is a collection of traffic elements, such as lanes, road boundary
// etc.
// It provides general information about the road.
message Road {
  optional Id id = 1;
  repeated RoadSection section = 2;

  // if lane road not in the junction, junction id is null.
  optional Id junction_id = 3;

  enum Type {
    UNKNOWN = 0;
    HIGHWAY = 1;
    CITY_ROAD = 2;
    PARK = 3;
  };
  optional Type type = 4;
}

```

Road（道路）用于描述一条完整道路的结构和属性，是高精地图中的顶层交通元素。

**数据结构说明：**

-   **id**（Id类型）：道路的唯一标识符。
-   **section**（RoadSection类型，多个）：道路包含的分段信息，每个分段包含若干车道和边界。
-   **junction_id**（Id类型）：如果该道路属于某个路口，则为对应路口的ID，否则为null。
-   **type**（Type枚举）：道路类型，包括：
    -   UNKNOWN：未知类型
    -   HIGHWAY：高速公路
    -   CITY_ROAD：城市道路
    -   PARK：园区道路

### 15. PNC Junction

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_geometry.proto";
import "modules/common_msgs/map_msgs/map_id.proto";

message Passage {
  optional Id id = 1;

  repeated Id signal_id = 2;
  repeated Id yield_id = 3;
  repeated Id stop_sign_id = 4;
  repeated Id lane_id = 5;

  enum Type {
    UNKNOWN = 0;
    ENTRANCE = 1;
    EXIT = 2;
  };
  optional Type type = 6;
};

message PassageGroup {
  optional Id id = 1;

  repeated Passage passage = 2;
};
// planning and control
message PNCJunction {
  optional Id id = 1;

  optional Polygon polygon = 2;

  repeated Id overlap_id = 3;

  repeated PassageGroup passage_group = 4;
}

```

PNCJunction（PNC路口）用于描述规划与控制（Planning and Control）相关的特殊路口区域，是Apollo高精地图中为自动驾驶决策提供支持的结构。

**数据结构说明：**

-   **id**（Id类型）：PNC路口的唯一标识符。
-   **polygon**（Polygon类型）：PNC路口的几何形状和边界。
-   **overlap_id**（多个Id类型）：与该PNC路口重叠的其他地图元素的ID列表。
-   **passage_group**（PassageGroup类型，多个）：通行组集合，每个通行组包含若干通行通道（Passage），用于描述路口内的可行驶路径。

PassageGroup和Passage结构可进一步细分路口内的信号灯、让行标志、停车标志、车道等元素，便于自动驾驶系统在复杂路口场景下进行路径规划和决策。

### 16. Map

```protobuf
syntax = "proto2";

package apollo.hdmap;

import "modules/common_msgs/map_msgs/map_clear_area.proto";
import "modules/common_msgs/map_msgs/map_crosswalk.proto";
import "modules/common_msgs/map_msgs/map_junction.proto";
import "modules/common_msgs/map_msgs/map_lane.proto";
import "modules/common_msgs/map_msgs/map_overlap.proto";
import "modules/common_msgs/map_msgs/map_parking_space.proto";
import "modules/common_msgs/map_msgs/map_pnc_junction.proto";
import "modules/common_msgs/map_msgs/map_road.proto";
import "modules/common_msgs/map_msgs/map_rsu.proto";
import "modules/common_msgs/map_msgs/map_signal.proto";
import "modules/common_msgs/map_msgs/map_speed_bump.proto";
import "modules/common_msgs/map_msgs/map_stop_sign.proto";
import "modules/common_msgs/map_msgs/map_yield_sign.proto";

// This message defines how we project the ellipsoidal Earth surface to a plane.
message Projection {
  // PROJ.4 setting:
  // "+proj=tmerc +lat_0={origin.lat} +lon_0={origin.lon} +k={scale_factor}
  // +ellps=WGS84 +no_defs"
  optional string proj = 1;
}

message Header {
  optional bytes version = 1;
  optional bytes date = 2;
  optional Projection projection = 3;
  // district area
  optional bytes district = 4;
  optional bytes generation = 5;
  optional bytes rev_major = 6;
  optional bytes rev_minor = 7;
  optional double left = 8;
  optional double top = 9;
  optional double right = 10;
  optional double bottom = 11;
  optional bytes vendor = 12;
}

message Map {
  optional Header header = 1;

  repeated Crosswalk crosswalk = 2;
  repeated Junction junction = 3;
  repeated Lane lane = 4;
  repeated StopSign stop_sign = 5;
  repeated Signal signal = 6;
  repeated YieldSign yield = 7;
  repeated Overlap overlap = 8;
  repeated ClearArea clear_area = 9;
  repeated SpeedBump speed_bump = 10;
  repeated Road road = 11;
  repeated ParkingSpace parking_space = 12;
  repeated PNCJunction pnc_junction = 13;
  repeated RSU rsu = 14;
}

```

Map（地图）是Apollo高精地图的顶层数据结构，包含了所有地图要素的集合。

**数据结构说明：**

-   **header**（Header类型）：地图的头部信息，包括版本、生成日期、投影方式、边界等元数据。
-   **crosswalk**（Crosswalk类型，多个）：人行横道元素集合。
-   **junction**（Junction类型，多个）：路口元素集合。
-   **lane**（Lane类型，多个）：车道元素集合。
-   **stop_sign**（StopSign类型，多个）：停车标志元素集合。
-   **signal**（Signal类型，多个）：交通信号灯元素集合。
-   **yield**（YieldSign类型，多个）：让行标志元素集合。
-   **overlap**（Overlap类型，多个）：重叠区域元素集合。
-   **clear_area**（ClearArea类型，多个）：禁停区元素集合。
-   **speed_bump**（SpeedBump类型，多个）：减速带元素集合。
-   **road**（Road类型，多个）：道路元素集合。
-   **parking_space**（ParkingSpace类型，多个）：停车位元素集合。
-   **pnc_junction**（PNCJunction类型，多个）：PNC路口元素集合。
-   **rsu**（RSU类型，多个）：路侧单元元素集合。

该结构将所有地图要素统一组织，便于自动驾驶系统进行全局地图数据的加载、查询和管理。



Reference:

