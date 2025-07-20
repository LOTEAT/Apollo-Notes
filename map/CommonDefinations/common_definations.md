<!--
 * @Author: LOTEAT
 * @Date: 2025-07-16 21:16:57
-->

## Common Definations详解

[知乎链接]()

[Github]()

本篇博客主要介绍一下apollo中实现的一些common definations，这些才是真正供后续模块所使用的。

### 1. ObjectWithAABox
```cpp
template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const apollo::common::math::AABox2d &aabox,
                  const Object *object, const GeoObject *geo_object,
                  const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const apollo::common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const apollo::common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const apollo::common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  apollo::common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};
```

`ObjectWithAABox`是一个模板类，用于将地图对象与其AABox关联起来，实现空间索引和快速查询功能。在这个类中，它封装了原始对象指针、几何对象指针、包围盒和ID。其中，`object`是对象的类型；`geo_object`是其几何类型。


### 2. Boundary
```cpp
struct LineBoundary {
  std::vector<apollo::common::PointENU> line_points;
};
struct PolygonBoundary {
  std::vector<apollo::common::PointENU> polygon_points;
};
```

这两个结构定义了地图中的边界表示方式：`LineBoundary`用点序列表示线性边界（如道路边界线、车道分隔线等），`PolygonBoundary`用点序列表示多边形边界（如区域边界、停车场轮廓等），都基于PointENU坐标系统来描述几何形状。


### 3. Polygon
```cpp
enum class PolygonType {
  JUNCTION_POLYGON = 0,
  PARKINGSPACE_POLYGON = 1,
  ROAD_HOLE_POLYGON = 2,
};

struct RoiAttribute {
  PolygonType type;
  Id id;
};

struct PolygonRoi {
  apollo::common::math::Polygon2d polygon;
  RoiAttribute attribute;
};
```

这段代码定义了地图中感兴趣区域（ROI）的多边形表示：`PolygonType`枚举定义了三种多边形类型（路口、停车位、道路孔洞），`RoiAttribute`结构包含多边形的类型和ID属性，`PolygonRoi`结构将具体的2D多边形几何形状与其属性信息组合，用于标识和管理地图中的特殊区域。

### 4. Defination
```cpp
using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, apollo::common::math::LineSegment2d>;
using LaneSegmentKDTree = apollo::common::math::AABoxKDTree2d<LaneSegmentBox>;
using OverlapInfoConstPtr = std::shared_ptr<const OverlapInfo>;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
using CrosswalkInfoConstPtr = std::shared_ptr<const CrosswalkInfo>;
using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
using YieldSignInfoConstPtr = std::shared_ptr<const YieldSignInfo>;
using ClearAreaInfoConstPtr = std::shared_ptr<const ClearAreaInfo>;
using SpeedBumpInfoConstPtr = std::shared_ptr<const SpeedBumpInfo>;
using RoadInfoConstPtr = std::shared_ptr<const RoadInfo>;
using ParkingSpaceInfoConstPtr = std::shared_ptr<const ParkingSpaceInfo>;
using RoadROIBoundaryPtr = std::shared_ptr<RoadROIBoundary>;
using PolygonRoiPtr = std::shared_ptr<PolygonRoi>;
using RoadRoiPtr = std::shared_ptr<RoadRoi>;
using PNCJunctionInfoConstPtr = std::shared_ptr<const PNCJunctionInfo>;
using RSUInfoConstPtr = std::shared_ptr<const RSUInfo>;
```
这里是都是一些定义上的东西。



### 5. Utility Function
```cpp
// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;

// Margin for comparation
constexpr double kEpsilon = 0.1;
void RemoveDuplicates(std::vector<Vec2d> *points) {
  RETURN_IF_NULL(points);

  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (const auto &point : *points) {
    if (count == 0 || point.DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = point;
    }
  }
  points->resize(count);
}

void PointsFromCurve(const Curve &input_curve, std::vector<Vec2d> *points) {
  RETURN_IF_NULL(points);
  points->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    } else {
      AERROR << "Can not handle curve type.";
    }
  }
  RemoveDuplicates(points);
}

apollo::common::math::Polygon2d ConvertToPolygon2d(const Polygon &polygon) {
  std::vector<Vec2d> points;
  points.reserve(polygon.point_size());
  for (const auto &point : polygon.point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  while (points.size() >= 2 && points[0].DistanceTo(points.back()) <=
                                   apollo::common::math::kMathEpsilon) {
    points.pop_back();
  }
  return apollo::common::math::Polygon2d(points);
}

void SegmentsFromCurve(
    const Curve &curve,
    std::vector<apollo::common::math::LineSegment2d> *segments) {
  RETURN_IF_NULL(segments);

  std::vector<Vec2d> points;
  PointsFromCurve(curve, &points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    segments->emplace_back(points[i], points[i + 1]);
  }
}

PointENU PointFromVec2d(const Vec2d &point) {
  PointENU pt;
  pt.set_x(point.x());
  pt.set_y(point.y());
  return pt;
}
```

以上是一些工具函数：`RemoveDuplicates`去除重复点以优化几何数据，`PointsFromCurve`从曲线中提取点序列，`ConvertToPolygon2d`将地图多边形转换为数学库的2D多边形，`SegmentsFromCurve`将曲线分解为线段序列，`PointFromVec2d`在不同坐标类型间转换，这些函数是地图几何处理的基础工具。



### 6. LaneInfo
```cpp
class LaneInfo {
 public:
  explicit LaneInfo(const Lane &lane);

  const Id &id() const { return lane_.id(); }
  const Id &road_id() const { return road_id_; }
  const Id &section_id() const { return section_id_; }
  const Lane &lane() const { return lane_; }
  const std::vector<apollo::common::math::Vec2d> &points() const {
    return points_;
  }
  const std::vector<apollo::common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  double Heading(const double s) const;
  double Curvature(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return overlaps_; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return cross_lanes_;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return signals_; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return yield_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return stop_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return crosswalks_;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return junctions_;
  }
  const std::vector<OverlapInfoConstPtr> &clear_areas() const {
    return clear_areas_;
  }
  const std::vector<OverlapInfoConstPtr> &speed_bumps() const {
    return speed_bumps_;
  }
  const std::vector<OverlapInfoConstPtr> &parking_spaces() const {
    return parking_spaces_;
  }
  const std::vector<OverlapInfoConstPtr> &pnc_junctions() const {
    return pnc_junctions_;
  }
  double total_length() const { return total_length_; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return sampled_left_width_;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return sampled_right_width_;
  }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  const std::vector<SampledWidth> &sampled_left_road_width() const {
    return sampled_left_road_width_;
  }
  const std::vector<SampledWidth> &sampled_right_road_width() const {
    return sampled_right_road_width_;
  }
  void GetRoadWidth(const double s, double *left_width,
                    double *right_width) const;
  double GetRoadWidth(const double s) const;

  bool IsOnLane(const apollo::common::math::Vec2d &point) const;
  bool IsOnLane(const apollo::common::math::Box2d &box) const;

  apollo::common::PointENU GetSmoothPoint(double s) const;
  double DistanceTo(const apollo::common::math::Vec2d &point) const;
  double DistanceTo(const apollo::common::math::Vec2d &point,
                    apollo::common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  apollo::common::PointENU GetNearestPoint(
      const apollo::common::math::Vec2d &point, double *distance) const;
  bool GetProjection(const apollo::common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;
  bool GetProjection(const apollo::common::math::Vec2d &point,
                     const double heading, double *accumulate_s,
                     double *lateral) const;

 private:
  friend class HDMapImpl;
  friend class RoadInfo;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);
  double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                            const double s) const;
  void CreateKDTree();
  void set_road_id(const Id &road_id) { road_id_ = road_id; }
  void set_section_id(const Id &section_id) { section_id_ = section_id; }

 private:
  const Lane &lane_;
  std::vector<apollo::common::math::Vec2d> points_;
  std::vector<apollo::common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<std::string> overlap_ids_;
  std::vector<OverlapInfoConstPtr> overlaps_;
  std::vector<OverlapInfoConstPtr> cross_lanes_;
  std::vector<OverlapInfoConstPtr> signals_;
  std::vector<OverlapInfoConstPtr> yield_signs_;
  std::vector<OverlapInfoConstPtr> stop_signs_;
  std::vector<OverlapInfoConstPtr> crosswalks_;
  std::vector<OverlapInfoConstPtr> junctions_;
  std::vector<OverlapInfoConstPtr> clear_areas_;
  std::vector<OverlapInfoConstPtr> speed_bumps_;
  std::vector<OverlapInfoConstPtr> parking_spaces_;
  std::vector<OverlapInfoConstPtr> pnc_junctions_;
  double total_length_ = 0.0;
  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;

  std::vector<SampledWidth> sampled_left_road_width_;
  std::vector<SampledWidth> sampled_right_road_width_;

  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  Id road_id_;
  Id section_id_;
};
```
以上是 `LaneInfo`的声明部分，从这个代码可以看出，其实`LaneInfo`就是对Lane属性和功能的一些封装。


#### 6.1 Init
```cpp
void LaneInfo::Init() {
  PointsFromCurve(lane_.central_curve(), &points_);
  CHECK_GE(points_.size(), 2U);
  segments_.clear();
  accumulated_s_.clear();
  unit_directions_.clear();
  headings_.clear();

  double s = 0;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);
    accumulated_s_.push_back(s);
    unit_directions_.push_back(segments_.back().unit_direction());
    s += segments_.back().length();
  }

  accumulated_s_.push_back(s);
  total_length_ = s;
  ACHECK(!unit_directions_.empty());
  unit_directions_.push_back(unit_directions_.back());
  for (const auto &direction : unit_directions_) {
    headings_.push_back(direction.Angle());
  }
  for (const auto &overlap_id : lane_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id.id());
  }
  ACHECK(!segments_.empty());

  sampled_left_width_.clear();
  sampled_right_width_.clear();
  for (const auto &sample : lane_.left_sample()) {
    sampled_left_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_sample()) {
    sampled_right_width_.emplace_back(sample.s(), sample.width());
  }

  if (lane_.has_type()) {
    if (lane_.type() == Lane::CITY_DRIVING) {
      for (const auto &p : sampled_left_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          AERROR
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_left_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
      for (const auto &p : sampled_right_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          AERROR
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_right_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
    } else if (lane_.type() == Lane::NONE) {
      AERROR << "lane_[id = " << lane_.id().DebugString() << "] type is NONE.";
    }
  } else {
    AERROR << "lane_[id = " << lane_.id().DebugString() << "] has NO type.";
  }

  sampled_left_road_width_.clear();
  sampled_right_road_width_.clear();
  for (const auto &sample : lane_.left_road_sample()) {
    sampled_left_road_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_road_sample()) {
    sampled_right_road_width_.emplace_back(sample.s(), sample.width());
  }

  CreateKDTree();
}
```

`LaneInfo::Init`函数是构造函数中调用的函数，本质上等同于构造函数，似乎合并到构造函数中更合适一些。这个函数负责初始化车道的几何信息和属性数据：首先从车道中心线提取点序列并构建线段，计算累积距离、单位方向向量和朝向角，然后处理车道宽度采样数据（包括车道宽度和道路宽度），并对城市驾驶车道进行宽度安全检查，最后创建KDTree以支持高效的空间查询。代码很简单，耐心一点看就能看明白。

#### 6.2 GetWidth
```cpp
void LaneInfo::GetWidth(const double s, double *left_width,
                        double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_width_, s);
  }
}

double LaneInfo::GetEffectiveWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return 2 * std::min(left_width, right_width);
}

double LaneInfo::GetWidthFromSample(
    const std::vector<LaneInfo::SampledWidth> &samples, const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= samples[0].first) {
    return samples[0].second;
  }
  if (s >= samples.back().first) {
    return samples.back().second;
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  while (low + 1 < high) {
    const int mid = (low + high) >> 1;
    if (samples[mid].first <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const LaneInfo::SampledWidth &sample1 = samples[low];
  const LaneInfo::SampledWidth &sample2 = samples[high];
  const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
  return sample1.second * ratio + sample2.second * (1.0 - ratio);
}
```

`GetWidth`函数根据给定的s坐标获取车道在该位置的左右宽度：通过调用`GetWidthFromSample`函数，使用二分查找定位目标位置在采样数据中的区间，然后进行线性插值计算精确的宽度值。`GetEffectiveWidth`就是取最短的一侧乘2估计道路长度。

#### 6.3 GetRoadWidth
```cpp
double LaneInfo::GetRoadWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

void LaneInfo::GetRoadWidth(const double s, double *left_width,
                            double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_road_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_road_width_, s);
  }
}

double LaneInfo::GetRoadWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(s, &left_width, &right_width);
  return left_width + right_width;
}
```

`GetRoadWidth`函数用于获取道路在指定s坐标位置的宽度信息：与`GetWidth`类似，但使用道路宽度采样数据而非车道宽度数据，提供两个重载版本分别返回总宽度和分别获取左右宽度，使用相同的二分查找和线性插值算法。

#### 6.4 Heading
```cpp
double LaneInfo::Heading(const double s) const {
  if (accumulated_s_.empty()) {
    return 0.0;
  }
  const double kEpsilon = 0.001;
  if (s + kEpsilon < accumulated_s_.front()) {
    AWARN << "s:" << s << " should be >= " << accumulated_s_.front();
    return headings_.front();
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    AWARN << "s:" << s << " should be <= " << accumulated_s_.back();
    return headings_.back();
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0 || *iter - s <= common::math::kMathEpsilon) {
    return headings_[index];
  }
  return common::math::slerp(headings_[index - 1], accumulated_s_[index - 1],
                             headings_[index], accumulated_s_[index], s);
}
```

`Heading`函数根据给定的s坐标获取车道在该位置的朝向角：首先进行边界检查，如果s超出范围则返回端点朝向并发出警告，然后使用二分查找定位目标位置，如果正好在采样点上则直接返回，否则使用球面线性插值(slerp)在相邻两个朝向角之间进行平滑插值，确保角度变化的连续性。

#### 6.5 Curvature
```cpp
double LaneInfo::Curvature(const double s) const {
  if (points_.size() < 2U) {
    AERROR << "Not enough points to compute curvature.";
    return 0.0;
  }
  const double kEpsilon = 0.001;
  if (s + kEpsilon < accumulated_s_.front()) {
    AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s > accumulated_s_.back() + kEpsilon) {
    AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (iter == accumulated_s_.end()) {
    ADEBUG << "Reach the end of lane.";
    return 0.0;
  }
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0) {
    ADEBUG << "Reach the beginning of lane";
    return 0.0;
  }
  return (headings_[index] - headings_[index - 1]) /
         (accumulated_s_[index] - accumulated_s_[index - 1] + kEpsilon);
}
```

`Curvature`函数计算车道在指定s坐标位置的曲率值：首先进行数据有效性检查和边界检查，然后使用二分查找定位目标位置。曲率的计算是$\frac{d\theta}{ds}$，也就是单位弧长下的方向变化率。但是由于我们无法微分，所以用差值去拟合微分，也就是代码的最后一行。

#### 6.6 GetProjection
```cpp
bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s,
                             double *lateral) const {
  RETURN_VAL_IF_NULL(accumulate_s, false);
  RETURN_VAL_IF_NULL(lateral, false);

  if (segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}

bool LaneInfo::GetProjection(const Vec2d &point, const double heading,
                             double *accumulate_s, double *lateral) const {
  RETURN_VAL_IF_NULL(accumulate_s, false);
  RETURN_VAL_IF_NULL(lateral, false);

  if (segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {
    if (abs(common::math::AngleDiff(segments_[i].heading(), heading)) >= M_PI_2)
      continue;
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}
```