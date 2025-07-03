<!-- filepath: c:\downloads\Apollo-Notes\map\MapMessages\map_messages.md -->
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

- **id**: 字符串类型的唯一标识符，用于标识和引用地图中的各种对象

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
- **PointENU**: 地图参考系坐标点，使用UTM投影，x(东)、y(北)、z(高度)
- **PointLLH**: 全球坐标系点，使用经度(lon)、纬度(lat)、高度(height)
- **Point2D/Point3D**: 通用2D/3D坐标点，具体含义根据上下文确定

#### 其他几何类型：
- **Quaternion**: 四元数，表示空间旋转
- **Polygon**: 多边形，由多个3D点组成（逆时针顺序）

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

#### 基础几何结构：
- **Polygon**: 多边形，由多个PointENU点组成
- **LineSegment**: 直线段，由一系列PointENU点构成
- **CurveSegment**: 曲线段，目前只支持直线段类型
  - 使用oneof预留扩展性
  - 包含起始位置、方向、长度等参数
- **Curve**: 复合曲线，由多个CurveSegment组成

这些结构是构建车道、道路边界、停车线等地图元素的基础几何组件。

