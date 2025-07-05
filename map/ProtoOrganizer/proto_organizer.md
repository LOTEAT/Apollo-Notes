<!--
 * @Author: LOTEAT
 * @Date: 2025-07-05 20:16:24
-->
## Proto Orginazer详解

[知乎链接]()

[Github]()

在XML地图的解析中，其实已经出现过了`ProtoOrganizer`，这里对其结构进行详解。

### 1. Internal结构体

在Apollo地图模块中，有一系列将Protobuf封装的中间件结构体，这些结构体末尾统一使用了Internal。这一类Internal结构体非常简单。

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


