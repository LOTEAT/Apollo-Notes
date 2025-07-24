<!--
 * @Author: LOTEAT
 * @Date: 2025-07-05 20:16:24
-->

## Proto Orginazer详解

[知乎链接](https://zhuanlan.zhihu.com/p/1925574451619169650)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/map/ProtoOrganizer/proto_organizer.md)

在XML地图的解析中，其实已经出现过了`ProtoOrganizer`，这里对其结构进行详解。

### 1. ProtoData Struct

```cpp
struct ProtoData {
  PbHeader header;
  std::unordered_map<std::string, PbLane> pb_lanes;
  std::unordered_map<std::string, PbRoad> pb_roads;
  std::unordered_map<std::string, PbCrosswalk> pb_crosswalks;
  std::unordered_map<std::string, PbClearArea> pb_clear_areas;
  std::unordered_map<std::string, PbSpeedBump> pb_speed_bumps;
  std::unordered_map<std::string, PbJunction> pb_junction;
  std::unordered_map<std::string, PbSignal> pb_signals;
  std::unordered_map<std::string, PbStopSign> pb_stop_signs;
  std::unordered_map<std::string, PbYieldSign> pb_yield_signs;
  std::unordered_map<std::string, PbOverlap> pb_overlaps;
  std::unordered_map<std::string, PbJunction> pb_junctions;
  std::unordered_map<std::string, StopLineInternal> pb_stop_lines;
  std::unordered_map<std::string, PbParkingSpace> pb_parking_spaces;
  std::unordered_map<std::string, PbPNCJunction> pb_pnc_junctions;
  std::unordered_map<std::string, PbRSU> pb_rsus;
};

```

`ProtoData` 是 Apollo 地图解析与组织过程中用于集中存储各类 Protobuf 地图元素的结构体。其主要作用是将解析得到的所有地图原始数据（如车道、道路、信号灯、停车标志、路口、重叠区等）以哈希表（unordered_map）的形式统一管理，便于后续的查找、组织和处理。

#### 主要字段说明：

-   `PbHeader header`：地图文件头信息。
-   `std::unordered_map<std::string, PbLane> pb_lanes`：所有车道对象，key为车道ID。
-   `std::unordered_map<std::string, PbRoad> pb_roads`：所有道路对象，key为道路ID。
-   `std::unordered_map<std::string, PbCrosswalk> pb_crosswalks`：所有人行横道对象。
-   `std::unordered_map<std::string, PbClearArea> pb_clear_areas`：所有清除区对象。
-   `std::unordered_map<std::string, PbSpeedBump> pb_speed_bumps`：所有减速带对象。
-   `std::unordered_map<std::string, PbJunction> pb_junction`：所有路口对象。
-   `std::unordered_map<std::string, PbSignal> pb_signals`：所有信号灯对象。
-   `std::unordered_map<std::string, PbStopSign> pb_stop_signs`：所有停车标志对象。
-   `std::unordered_map<std::string, PbYieldSign> pb_yield_signs`：所有让行标志对象。
-   `std::unordered_map<std::string, PbOverlap> pb_overlaps`：所有重叠区对象。
-   `std::unordered_map<std::string, PbJunction> pb_junctions`：所有路口对象。
-   `std::unordered_map<std::string, StopLineInternal> pb_stop_lines`：所有停止线对象。
-   `std::unordered_map<std::string, PbParkingSpace> pb_parking_spaces`：所有停车位对象。
-   `std::unordered_map<std::string, PbPNCJunction> pb_pnc_junctions`：所有PNC路口对象。
-   `std::unordered_map<std::string, PbRSU> pb_rsus`：所有路侧单元对象。

### 2. ProtoOrganizer

```cpp
class ProtoOrganizer {
 public:
  void GetRoadElements(std::vector<RoadInternal>* roads);
  void GetJunctionElements(const std::vector<JunctionInternal>& junctions);
  void GetOverlapElements(const std::vector<RoadInternal>& roads,
                          const std::vector<JunctionInternal>& junctions);
  void GetObjectElements(const ObjectInternal& objects);
  void OutputData(apollo::hdmap::Map* pb_map);

 private:
  void GetLaneObjectOverlapElements(
      const std::string& lane_id,
      const std::vector<OverlapWithLane>& overlap_with_lanes);
  void GetLaneSignalOverlapElements(
      const std::string& lane_id,
      const std::vector<OverlapWithLane>& overlap_with_lanes);
  void GetLaneJunctionOverlapElements(
      const std::string& lane_id,
      const std::vector<OverlapWithLane>& overlap_with_lanes);
  void GetLaneLaneOverlapElements(
      const std::unordered_map<std::pair<std::string, std::string>,
                               OverlapWithLane, apollo::common::util::PairHash>&
          lane_lane_overlaps);
  void GetJunctionObjectOverlapElements(
      const std::vector<JunctionInternal>& junctions);

 private:
  ProtoData proto_data_;
};
```

`ProtoOrganizer` 是 Apollo 地图模块中用于组织和输出结构化地图数据的核心类。其主要职责是：

-   将解析得到的原始 Protobuf 地图元素（见 ProtoData）进一步结构化，生成 RoadInternal、JunctionInternal、ObjectInternal 等中间结构体。
-   负责各类地图元素（道路、路口、重叠区、对象等）的组织、整合与输出。
-   最终将结构化后的数据输出为 Apollo hdmap 的标准 Protobuf 格式。

有一点需要注意的是，这里所有接口用的都是Get，但是我觉得改成Set更为妥当一些。

#### 主要接口说明：

-   `void GetRoadElements(std::vector<RoadInternal>* roads);`
    -   组织并生成所有道路及其子元素的结构化信息。

-   `void GetJunctionElements(const std::vector<JunctionInternal>& junctions);`
    -   处理所有路口相关的结构体信息。

-   `void GetOverlapElements(const std::vector<RoadInternal>& roads, const std::vector<JunctionInternal>& junctions);`
    -   处理道路与路口之间的重叠关系。

-   `void GetObjectElements(const ObjectInternal& objects);`
    -   处理与地图对象相关的结构体信息。

-   `void OutputData(apollo::hdmap::Map* pb_map);`
    -   将组织好的结构化数据输出为 Apollo hdmap 的 Protobuf 地图对象。

#### 内部辅助函数：

-   `GetLaneObjectOverlapElements`、`GetLaneSignalOverlapElements`、`GetLaneJunctionOverlapElements`、`GetLaneLaneOverlapElements`、`GetJunctionObjectOverlapElements` 等函数，分别处理车道与对象、信号、路口、其他车道等的重叠关系。

#### 数据成员：

-   `ProtoData proto_data_`：存储所有原始 Protobuf 地图元素的数据结构。

这里就不再过多赘述`ProtoOrganizer`的具体实现细节了，本质上就是将Pb数据读取到Internal数据中。至此，apollo的地图数据加载内容就结束了。有兴趣的可以尝试一下`modules/map/tools`下的各类工具代码，尝试下xml文件和bin文件的互转。



Reference:

