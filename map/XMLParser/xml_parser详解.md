<!--
 * @Author: LOTEAT
 * @Date: 2025-07-03 18:52:20
-->
## XML Parser详解

[知乎链接]()

[Github]()

Apollo的高精地图数据通常采用XML格式进行存储和描述。XML解析模块的主要作用是将这些结构化的地图数据文件高效、准确地解析为系统可用的数据结构，为自动驾驶各模块提供基础的地图信息支撑。想要理解清楚XML Parser，务必要清楚的了解这个地图的XML架构，否则看完这篇博客依旧会是云里雾里的。

### 1. OpendriveAdapter::LoadData
```cpp
bool OpendriveAdapter::LoadData(const std::string& filename,
                                apollo::hdmap::Map* pb_map) {
  CHECK_NOTNULL(pb_map);

  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    AERROR << "fail to load file " << filename;
    return false;
  }

  // root node
  const tinyxml2::XMLElement* root_node = document.RootElement();
  ACHECK(root_node != nullptr);
  // header
  PbHeader* map_header = pb_map->mutable_header();
  // parse header
  Status status = HeaderXmlParser::Parse(*root_node, map_header);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive header, " << status.error_message();
    return false;
  }

  // road
  std::vector<RoadInternal> roads;
  status = RoadsXmlParser::Parse(*root_node, &roads);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive road, " << status.error_message();
    return false;
  }

  // junction
  std::vector<JunctionInternal> junctions;
  status = JunctionsXmlParser::Parse(*root_node, &junctions);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive junction, " << status.error_message();
    return false;
  }

  // objects
  ObjectInternal objects;
  status = ObjectsXmlParser::ParseObjects(*root_node, &objects);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive objects, " << status.error_message();
    return false;
  }

  ProtoOrganizer proto_organizer;
  proto_organizer.GetRoadElements(&roads);
  proto_organizer.GetJunctionElements(junctions);
  proto_organizer.GetObjectElements(objects);
  proto_organizer.GetOverlapElements(roads, junctions);
  proto_organizer.OutputData(pb_map);

  return true;
}
```

该函数的主要流程如下：
1. 加载XML地图文件，并检查文件是否成功打开。
2. 获取XML的根节点，准备后续解析。
3. 解析header信息，填充到pb_map的header字段。
4. 依次解析road、junction、object等主要地图元素，遇到解析失败则直接返回false。
5. 通过ProtoOrganizer将解析得到的各类元素组织成最终的地图数据结构，并输出到pb_map。
6. 全部流程无误后返回true，表示地图数据加载和解析成功。

### 2. Header Parser
```cpp
Status HeaderXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                              PbHeader* header) {
  // find header information
  auto header_node = xml_node.FirstChildElement("header");
  if (!header_node) {
    std::string err_msg = "xml data missing header";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  // set value
  std::string rev_major;
  std::string rev_minor;
  std::string database_name;
  std::string version;
  std::string date;
  double north = 0.0;
  double south = 0.0;
  double west = 0.0;
  double east = 0.0;
  std::string vendor;
  int checker =
      UtilXmlParser::QueryStringAttribute(*header_node, "revMajor", &rev_major);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "revMinor", &rev_minor);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "name", &database_name);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "version", &version);
  checker += UtilXmlParser::QueryStringAttribute(*header_node, "date", &date);
  checker += header_node->QueryDoubleAttribute("north", &north);
  checker += header_node->QueryDoubleAttribute("south", &south);
  checker += header_node->QueryDoubleAttribute("east", &east);
  checker += header_node->QueryDoubleAttribute("west", &west);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "vendor", &vendor);
  // XML_SUCCESS = 0
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parsing header attributes";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  // geo reference information
  auto geo_reference_node = header_node->FirstChildElement("geoReference");
  if (!geo_reference_node) {
    std::string err_msg = "Error parsing header geoReoference attributes";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  auto geo_text = geo_reference_node->FirstChild()->ToText();
  if (!geo_text) {
    std::string err_msg = "Error parsing header geoReoference text";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  // coordinate frame
  std::string zone_id;
  std::string from_coordinate = geo_text->Value();
  auto projection_node = header_node->FirstChildElement("projection");
  if (projection_node != nullptr) {
    // utm information
    auto utm_node = projection_node->FirstChildElement("utm");
    if (!utm_node) {
      std::string err_msg = "Error parsing header utm node";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    checker =
        UtilXmlParser::QueryStringAttribute(*utm_node, "zoneID", &zone_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing utm zone id attributes";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
  } else {
    // longitude zone id
    int eastZone = GetLongZone(east);
    int westZone = GetLongZone(west);
    if (eastZone != westZone) {
      std::string err_msg = "unsupport data in more than one zones";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    zone_id = std::to_string(westZone);
  }

  std::string to_coordinate =
      absl::StrCat("+proj=utm +zone=", zone_id,
                   " +ellps=WGS84 +datum=WGS84 +units=m +no_defs");
  CoordinateConvertTool::GetInstance()->SetConvertParam(from_coordinate,
                                                        to_coordinate);
  // set meta information
  header->set_version(version);
  header->set_date(date);
  header->mutable_projection()->set_proj(to_coordinate);
  header->set_district(database_name);
  header->set_rev_major(rev_major);
  header->set_rev_minor(rev_minor);
  header->set_left(west);
  header->set_right(east);
  header->set_top(north);
  header->set_bottom(south);
  header->set_vendor(vendor);

  return Status::OK();
}
```

该Header Parser的主要作用是解析XML地图文件中的<header>标签，将其各项属性和地理参考信息提取并填充到地图头部数据结构中。其流程包括：
1. 查找<header>节点，若缺失则报错返回。
2. 依次读取revMajor、revMinor、name、version、date、north、south、east、west、vendor等属性。
3. 解析<geoReference>子节点，获取原始坐标系描述。
4. 判断是否有<projection>子节点，若有则进一步解析UTM分区信息，否则根据经纬度自动推算zone id。
5. 组装目标坐标系字符串，并设置全局坐标转换参数。
6. 最后将所有解析到的元数据写入header对象。

这里会有的知识盲区可能UTM坐标系和WGS84坐标系，这里简单查阅一下资料，只需要掌握两者转换的库proj的简单用法即可。

### 3. Road Parser
```cpp
Status RoadsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                             std::vector<RoadInternal>* roads) {
  CHECK_NOTNULL(roads);

  auto road_node = xml_node.FirstChildElement("road");
  while (road_node) {
    // road attributes
    std::string id;
    std::string junction_id;
    int checker = UtilXmlParser::QueryStringAttribute(*road_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*road_node, "junction",
                                                   &junction_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing road attributes";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    RoadInternal road_internal;
    road_internal.id = id;
    road_internal.road.mutable_id()->set_id(id);
    if (IsRoadBelongToJunction(junction_id)) {
      road_internal.road.mutable_junction_id()->set_id(junction_id);
    }

    std::string type;
    checker = UtilXmlParser::QueryStringAttribute(*road_node, "type", &type);
    if (checker != tinyxml2::XML_SUCCESS) {
      // forward compatibility with old data
      type = "CITYROAD";
    }
    PbRoadType pb_road_type;
    RETURN_IF_ERROR(to_pb_road_type(type, &pb_road_type));
    road_internal.road.set_type(pb_road_type);
    // lane -> object -> signal
    // lanes
    RETURN_IF_ERROR(LanesXmlParser::Parse(*road_node, road_internal.id,
                                          &road_internal.sections));

    // objects
    Parse_road_objects(*road_node, &road_internal);
    // signals
    Parse_road_signals(*road_node, &road_internal);

    roads->push_back(road_internal);
    road_node = road_node->NextSiblingElement("road");
  }

  return Status::OK();
}
```

Road Parser 主要负责解析XML地图文件中的<road>标签，将道路的结构、属性和包含的车道等信息提取出来，组织成系统可用的数据结构。其典型流程包括：
1. 查找并遍历所有<road>节点。
2. 解析每条道路的基本属性（如id、类型、关联路口等）。
3. 解析每个<road>下的`lanes`、`objects`、`signals`等子标签，提取车道分布、边界、限速等详细信息。这里对于objects和signal的解析，认真慢慢看源代码应该就可以看明白。
4. 将解析得到的所有道路及其子元素组织成Road对象，供后续地图数据加载和自动驾驶模块使用。

注意这里对于Road是否为junction的判断，只需要判断Road中的junction tag，等于-1说明不是junction，反之则为junction。

### 4. Lanes Parser
```cpp
Status LanesXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                             const std::string& road_id,
                             std::vector<RoadSectionInternal>* sections) {
  CHECK_NOTNULL(sections);
  const auto lanes_node = xml_node.FirstChildElement("lanes");
  CHECK_NOTNULL(lanes_node);
  const tinyxml2::XMLElement* sub_node =
      lanes_node->FirstChildElement("laneSection");
  CHECK_NOTNULL(sub_node);

  size_t section_cnt = 0;
  while (sub_node) {
    RoadSectionInternal section_internal;
    std::string section_id = std::to_string(++section_cnt);
    section_internal.id = section_id;
    section_internal.section.mutable_id()->set_id(section_id);
    RETURN_IF_ERROR(ParseLaneSection(*sub_node, &section_internal.lanes));
    RETURN_IF_ERROR(ParseSectionBoundary(
        *sub_node,
        section_internal.section.mutable_boundary()->mutable_outer_polygon()));
    sections->push_back(section_internal);

    sub_node = sub_node->NextSiblingElement("laneSection");
  }

  CHECK_NE(sections->size(), 0U);

  return Status::OK();
}
```

Lanes Parser（LanesXmlParser::Parse）主要负责解析每条道路（road）下的<lanes>标签及其所有<laneSection>子标签，将道路的车道分段结构和边界信息提取出来，组织成可用的数据结构。

简要流程如下：
1. 首先查找当前road节点下的<lanes>标签，若不存在则报错。
2. 遍历<lanes>下的每一个<laneSection>标签，每个laneSection代表道路上的一个车道段。
3. 对每个laneSection：
   - 生成唯一的section_id，并初始化对应的RoadSectionInternal结构。之所以这里没有用地图的id tag，是因为LaneSection没有id tag。
   - 调用ParseLaneSection解析该段下的所有车道信息（如左、中、右各个lane的属性和几何信息）。
   - 调用ParseSectionBoundary解析该段的物理边界点信息。
   - 将解析结果加入sections列表。
4. 所有laneSection解析完成后，sections中即包含了该道路的全部车道段及其详细结构。

该解析器的作用是将XML中复杂的车道分段、车道属性、边界等信息，结构化为后续地图构建和自动驾驶模块可直接使用的数据对象。

```cpp
Status LanesXmlParser::ParseSectionBoundary(
    const tinyxml2::XMLElement& xml_node, PbBoundaryPolygon* boundary) {
  CHECK_NOTNULL(boundary);

  auto boundaries_node = xml_node.FirstChildElement("boundaries");
  if (boundaries_node == nullptr) {
    std::string err_msg = "Error parse boundaries";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  auto sub_node = boundaries_node->FirstChildElement("boundary");
  while (sub_node) {
    PbBoundaryEdge* boundary_edge = boundary->add_edge();
    RETURN_IF_ERROR(
        UtilXmlParser::ParseCurve(*sub_node, boundary_edge->mutable_curve()));
    std::string type;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "type", &type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse boundary type";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    PbBoundaryEdgeType boundary_type;
    RETURN_IF_ERROR(ToPbBoundaryType(type, &boundary_type));
    boundary_edge->set_type(boundary_type);

    sub_node = sub_node->NextSiblingElement("boundary");
  }

  return Status::OK();
}
```
这里用到的`ParseSectionBoundary`，其实就是将LaneSection的左右边界从WGS84转化为UTM坐标，具体细节可以研究一下`util_xml_parser.cc`这个文件中的函数，因为这个`ParseSectionBoundary`是通过`util_xml_parser.cc`中的各个函数进行坐标转换的。`util_xml_parser.cc`内的函数比较简单，这里就不赘述了。


### 5. LaneSection Parser
```cpp
Status LanesXmlParser::ParseLaneSection(const tinyxml2::XMLElement& xml_node,
                                        std::vector<LaneInternal>* lanes) {
  CHECK_NOTNULL(lanes);

  // left
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("left");
  if (sub_node) {
    sub_node = sub_node->FirstChildElement("lane");
    while (sub_node) {
      LaneInternal lane_internal;
      RETURN_IF_ERROR(ParseLane(*sub_node, &lane_internal));
      *(lane_internal.lane.mutable_left_boundary()) =
          lane_internal.lane.right_boundary();
      lane_internal.lane.clear_right_boundary();
      if (lanes->size() > 0) {
        PbLane& left_neighbor_lane = lanes->back().lane;
        *(left_neighbor_lane.mutable_right_boundary()) =
            lane_internal.lane.left_boundary();
      }
      lanes->push_back(lane_internal);
      sub_node = sub_node->NextSiblingElement("lane");
    }
  }

  // center
  LaneInternal reference_lane_internal;
  sub_node = xml_node.FirstChildElement("center");
  CHECK_NOTNULL(sub_node);
  sub_node = sub_node->FirstChildElement("lane");
  CHECK_NOTNULL(sub_node);
  RETURN_IF_ERROR(ParseLane(*sub_node, &reference_lane_internal));
  *(reference_lane_internal.lane.mutable_left_boundary()) =
      reference_lane_internal.lane.right_boundary();
  if (lanes->size() > 0) {
    PbLane& left_neighbor_lane = lanes->back().lane;
    *(left_neighbor_lane.mutable_right_boundary()) =
        reference_lane_internal.lane.left_boundary();
  }

  // right
  sub_node = xml_node.FirstChildElement("right");
  if (sub_node) {
    sub_node = sub_node->FirstChildElement("lane");
    PbLane* left_neighbor_lane = &reference_lane_internal.lane;
    while (sub_node) {
      // PbLane lane
      LaneInternal lane_internal;
      RETURN_IF_ERROR(ParseLane(*sub_node, &lane_internal));
      *(lane_internal.lane.mutable_left_boundary()) =
          left_neighbor_lane->right_boundary();
      lanes->push_back(lane_internal);
      left_neighbor_lane = &lanes->back().lane;
      sub_node = sub_node->NextSiblingElement("lane");
    }
  }
  return Status::OK();
}
```

LaneSection Parser（LanesXmlParser::ParseLaneSection）主要负责解析每条道路（road）下的<laneSection>标签，将道路的车道分段结构提取出来，组织成可用的数据结构。

简要流程如下：
1. 查找当前laneSection节点下的 &lt;left&gt;、&lt;center&gt;、&lt;right&gt;标签，分别对应车道的左侧、中心和右侧部分。
2. 对于 &lt;left&gt; 和&lt;right&gt;部分，遍历其下的每一个<lane>标签，解析出每条车道的详细信息，并根据相邻车道的边界信息设置左右边界。
3. 对于&lt;center&gt;部分，解析其下的<lane>标签，作为参考车道（reference lane），并设置其左右边界。
4. 将解析得到的所有车道段及其子元素组织成Lane对象，供后续地图数据加载和自动驾驶模块使用。

注意：
- 车道的左右边界在不同车道段间的传递和赋值关系较为复杂，需仔细理顺。
- 解析过程中遇到任何字段缺失或格式错误都会返回错误状态，保证数据完整性。
- 该解析器将XML中关于车道分段的描述，转换为系统内部一致的结构化数据，供后续处理使用。

### 6. Lane Parser

```cpp
Status LanesXmlParser::ParseLane(const tinyxml2::XMLElement& xml_node,
                                 LaneInternal* lane_internal) {
  CHECK_NOTNULL(lane_internal);

  PbLane* lane = &lane_internal->lane;
  // lane id
  int id = 0;
  int checker = xml_node.QueryIntAttribute("id", &id);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane id";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  std::string lane_id;
  checker = UtilXmlParser::QueryStringAttribute(xml_node, "uid", &lane_id);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane uid";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->mutable_id()->set_id(lane_id);

  // lane type
  std::string lane_type;
  checker = UtilXmlParser::QueryStringAttribute(xml_node, "type", &lane_type);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  PbLaneType pb_lane_type;
  Status success = ToPbLaneType(lane_type, &pb_lane_type);
  if (!success.ok()) {
    std::string err_msg = "Error convert lane type to pb lane type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->set_type(pb_lane_type);

  // border
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("border");
  if (sub_node) {
    PbLaneBoundary* lane_boundary = lane->mutable_right_boundary();
    ACHECK(lane_boundary != nullptr);
    success =
        UtilXmlParser::ParseCurve(*sub_node, lane_boundary->mutable_curve());
    if (!success.ok()) {
      std::string err_msg = "Error parse lane border";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    lane_boundary->set_length(
        UtilXmlParser::CurveLength(*lane_boundary->mutable_curve()));

    bool is_virtual = false;
    std::string virtual_border = "FALSE";
    checker = UtilXmlParser::QueryStringAttribute(*sub_node, "virtual",
                                                  &virtual_border);
    if (checker == tinyxml2::XML_SUCCESS) {
      if (virtual_border == "TRUE") {
        is_virtual = true;
      }
    }
    lane_boundary->set_virtual_(is_virtual);
  }

  // road mark
  if (sub_node) {
    sub_node = sub_node->FirstChildElement("borderType");
  }
  while (sub_node) {
    PbLaneBoundary* lane_boundary = lane->mutable_right_boundary();
    PbLaneBoundaryTypeType boundary_type;
    success = ParseLaneBorderMark(*sub_node, &boundary_type);
    if (!success.ok()) {
      std::string err_msg = "Error parse lane border type";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    double s_offset = 0.0;
    checker = sub_node->QueryDoubleAttribute("sOffset", &s_offset);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse lane boundary type offset.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    auto lane_boundary_type = lane_boundary->add_boundary_type();
    lane_boundary_type->set_s(s_offset);
    lane_boundary_type->add_types(boundary_type);
    sub_node = sub_node->NextSiblingElement("borderType");
  }

  // reference line
  if (IsReferenceLane(id)) {
    return Status::OK();
  }

  // turn type
  std::string turn_type;
  checker =
      UtilXmlParser::QueryStringAttribute(xml_node, "turnType", &turn_type);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane turn type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  PbTurnType pb_turn_type;
  success = ToPbTurnType(turn_type, &pb_turn_type);
  if (!success.ok()) {
    std::string err_msg = "Error convert turn type to pb turn type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->set_turn(pb_turn_type);

  // direction
  RETURN_IF_ERROR(ParseDirection(xml_node, lane));

  // link
  sub_node = xml_node.FirstChildElement("link");
  if (sub_node) {
    ParseLaneLink(*sub_node, lane);
  }

  // center curve
  RETURN_IF_ERROR(ParseCenterCurve(xml_node, lane));
  // speed
  RETURN_IF_ERROR(ParseSpeed(xml_node, lane));
  // sample association
  RETURN_IF_ERROR(ParseSampleAssociates(xml_node, lane));
  // road sample association
  RETURN_IF_ERROR(ParseRoadSampleAssociates(xml_node, lane));

  // overlap object
  ParseObjectOverlapGroup(xml_node, &lane_internal->overlap_objects);
  // overlap signal
  ParseSignalOverlapGroup(xml_node, &lane_internal->overlap_signals);
  // overlap junction
  ParseJunctionOverlapGroup(xml_node, &lane_internal->overlap_junctions);
  // overlap lane
  ParseLaneOverlapGroup(xml_node, &lane_internal->overlap_lanes);

  return Status::OK();
}
```

Lane Parser（LanesXmlParser::ParseLane）主要负责解析每个<lane>标签，将车道的详细属性和几何信息从XML节点提取并填充到LaneInternal结构体中。其主要流程和要点如下：

1. 解析车道ID和唯一标识（uid），并写入lane->id。
2. 解析车道类型（如普通车道、应急车道等），并转换为枚举类型写入lane->type。
3. 解析<border>标签，提取车道边界曲线（通常为右侧边界），并判断是否为虚拟边界（如虚线、不可通行线等），写入lane->right_boundary。
4. 解析<borderType>子标签，提取边界类型（如实线、虚线等）及其sOffset（距离起点的偏移），并写入boundary_type列表。
5. 判断是否为参考线（reference lane），若是则直接返回，reference lane 的id就是0。
6. 解析车道的turnType（转向类型，如直行、左转、右转等），并写入lane->turn。
7. 解析车道方向（direction）、车道连接信息（link）、中心线（center curve）、限速（speed）、采样点关联（sample associates）、道路采样点关联（road sample associates）等。
8. 解析与其他对象（object）、信号灯（signal）、路口（junction）、其他车道（lane）的重叠关系，并分别写入overlap_objects、overlap_signals、overlap_junctions、overlap_lanes。


车道边界的解析和赋值较为复杂，需要理清楚xml文件中的lane中存在的标签。

后面的都是对Overlap的解析，代码也相对比较简单。


### 7. Junction Parser
```cpp
Status JunctionsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                                 std::vector<JunctionInternal>* junctions) {
  const tinyxml2::XMLElement* junction_node =
      xml_node.FirstChildElement("junction");
  while (junction_node) {
    // id
    std::string junction_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*junction_node, "id", &junction_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse junction id";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    // outline
    const tinyxml2::XMLElement* sub_node =
        junction_node->FirstChildElement("outline");
    if (!sub_node) {
      std::string err_msg = "Error parse junction outline";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    PbJunction junction;
    // set id
    junction.mutable_id()->set_id(junction_id);
    PbPolygon* polygon = junction.mutable_polygon();
    RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));
    // not exactly understand
    JunctionInternal junction_internal;
    junction_internal.junction = junction;

    // overlap
    sub_node = junction_node->FirstChildElement("objectOverlapGroup");
    if (sub_node) {
      sub_node = sub_node->FirstChildElement("objectReference");
      while (sub_node) {
        std::string object_id;
        checker =
            UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
        if (checker != tinyxml2::XML_SUCCESS) {
          std::string err_msg = "Error parse junction overlap id";
          return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
        }

        OverlapWithJunction overlap_with_juntion;
        overlap_with_juntion.object_id = object_id;
        junction_internal.overlap_with_junctions.push_back(
            overlap_with_juntion);

        sub_node = sub_node->NextSiblingElement("objectReference");
      }
    }

    junctions->push_back(junction_internal);
    junction_node = junction_node->NextSiblingElement("junction");
  }
  return Status::OK();
}
```
这个就是对junction进行parse，核心就是读取属性，然后把junction的外部轮廓解析出来。