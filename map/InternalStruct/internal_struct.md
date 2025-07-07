<!--
 * @Author: LOTEAT
 * @Date: 2025-07-07 13:29:52
-->
## BBox详解

[知乎链接]()

[Github]()

本篇博客是介绍`ProtoOrganizer`的过渡篇，主要介绍一下apollo中实现的一系列中间结构体。

### 1. Internal结构体

在Apollo地图模块中，有一系列将Protobuf封装的中间件结构体，这些结构体末尾统一使用了Internal。这一类Internal结构体非常简单。

下面是protobuf生成类，使用Pb+Class进行表示。
```cpp
using PbHeader = apollo::hdmap::Header;
using PbRoad = apollo::hdmap::Road;
using PbRoadSection = apollo::hdmap::RoadSection;
using PbLane = apollo::hdmap::Lane;
using PbJunction = apollo::hdmap::Junction;
using PbSignal = apollo::hdmap::Signal;
using PbSubSignal = apollo::hdmap::Subsignal;
using PbRSU = apollo::hdmap::RSU;
using PbCrosswalk = apollo::hdmap::Crosswalk;
using PbParkingSpace = apollo::hdmap::ParkingSpace;
using PbSpeedBump = apollo::hdmap::SpeedBump;
using PbStopSign = apollo::hdmap::StopSign;
using PbYieldSign = apollo::hdmap::YieldSign;
using PbObjectOverlapInfo = apollo::hdmap::ObjectOverlapInfo;
using PbOverlap = apollo::hdmap::Overlap;
using PbClearArea = apollo::hdmap::ClearArea;
using PbLineSegment = apollo::hdmap::LineSegment;
using PbCurveSegment = apollo::hdmap::CurveSegment;
using PbCurve = apollo::hdmap::Curve;
using PbPoint3D = apollo::common::PointENU;
using PbLaneType = apollo::hdmap::Lane_LaneType;
using PbTurnType = apollo::hdmap::Lane_LaneTurn;
using PbID = apollo::hdmap::Id;
using PbLaneBoundary = apollo::hdmap::LaneBoundary;
using PbLaneBoundaryTypeType = apollo::hdmap::LaneBoundaryType_Type;
using PbPolygon = apollo::hdmap::Polygon;
using PbBoundaryPolygon = apollo::hdmap::BoundaryPolygon;
using PbBoundaryEdge = apollo::hdmap::BoundaryEdge;
using PbRegionOverlap = apollo::hdmap::RegionOverlapInfo;
using PbPNCJunction = apollo::hdmap::PNCJunction;

using PbLaneDirection = apollo::hdmap::Lane_LaneDirection;
using PbSignalType = apollo::hdmap::Signal_Type;
using PbSubSignalType = apollo::hdmap::Subsignal_Type;
using PbStopSignType = apollo::hdmap::StopSign_StopType;
using PbBoundaryEdgeType = apollo::hdmap::BoundaryEdge_Type;
using PbRoadType = apollo::hdmap::Road_Type;
using PbSignInfoType = apollo::hdmap::SignInfo::Type;
using PbPassageType = apollo::hdmap::Passage_Type;
using PbPassageGroup = apollo::hdmap::PassageGroup;
```

而Internal结构体，实际上就是对Pb类的封装或者是补充。比如`StopLineInternal`，是通过对id和curve封装实现的，虽然我觉得这个地方写成Pb类型也可以。
```cpp
struct StopLineInternal {
  std::string id;
  PbCurve curve;
};
```

### 2. Internal结构体详解

以下对主要Internal结构体进行简要说明：

- **StopLineInternal**
  - `std::string id`：停止线ID。
  - `PbCurve curve`：停止线的几何曲线。


- **StopSignInternal**
  - `std::string id`：停车标志ID。
  - `PbStopSign stop_sign`：对应的Protobuf停车标志对象。
  - `std::unordered_set<std::string> stop_line_ids`：关联的停止线ID集合。


- **YieldSignInternal**
  - `std::string id`：让行标志ID。
  - `PbYieldSign yield_sign`：对应的Protobuf让行标志对象。
  - `std::unordered_set<std::string> stop_line_ids`：关联的停止线ID集合。
  - 作用：描述让行标志及其关联的停止线。

- **TrafficLightInternal**
  - `std::string id`：信号灯ID。
  - `PbSignal traffic_light`：对应的Protobuf信号灯对象。
  - `std::unordered_set<std::string> stop_line_ids`：关联的停止线ID集合。


- **OverlapWithLane**
  - `std::string object_id`：重叠对象的ID，通常指与当前车道发生重叠的信号、物体、路口或其他车道的唯一标识。
  - `double start_s`：重叠区间在本车道上的起始位置（s坐标，单位为米）。
  - `double end_s`：重叠区间在本车道上的终止位置（s坐标，单位为米）。
  - `bool is_merge`：是否为合并区域（如多车道汇入、分流等场景）。
  - `std::string region_overlap_id`：若为合并区域，则记录合并区域的ID。
  - `std::vector<PbRegionOverlap> region_overlaps`：与本车道重叠的区域信息集合。

- **OverlapWithJunction**
  - `std::string object_id`：与路口发生重叠的对象ID。


- **LaneInternal**
  - `PbLane lane`：车道的Protobuf对象。
  - `std::vector<OverlapWithLane> overlap_signals/objects/junctions/lanes`：与信号、物体、路口、其他车道的重叠信息。

- **JunctionInternal**
  - `PbJunction junction`：路口的Protobuf对象。
  - `std::unordered_set<std::string> road_ids`：关联道路ID集合。
  - `std::vector<OverlapWithJunction> overlap_with_junctions`：与其他路口的重叠信息。

- **RoadSectionInternal**
  - `std::string id`：道路分段ID。
  - `PbRoadSection section`：道路分段的Protobuf对象。
  - `std::vector<LaneInternal> lanes`：包含的车道信息。

- **RoadInternal**
  - `std::string id`：道路ID。
  - `PbRoad road`：道路的Protobuf对象。
  - `bool in_junction`：是否属于路口。
  - `std::string junction_id`：关联路口ID。
  - `std::string type`：道路类型。
  - `std::vector<RoadSectionInternal> sections`：道路分段信息。
  - 以及与信号灯、停车标志、让行标志、人行横道、清除区、减速带、停止线、停车位、PNC路口等相关的元素集合。
  - 作用：综合描述一条道路及其所有子元素和关联关系。

- **RSUInternal**
  - `std::string id`：路侧单元ID。
  - `PbRSU rsu`：对应的Protobuf RSU对象。

- **ObjectInternal**
  - `std::vector<RSUInternal> rsus`：包含的RSU集合。
