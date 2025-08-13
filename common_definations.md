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

`Curvature`函数计算车道在指定s坐标位置的曲率值：首先进行数据有效性检查和边界检查，然后使用二分查找定位目标位置。曲率的计算是<img src="https://www.zhihu.com/equation?tex=%5Cfrac%7Bd%5Ctheta%7D%7Bds%7D" alt="\frac{d\theta}{ds}" class="ee_img tr_noresize" eeimg="1">，也就是单位弧长下的方向变化率。但是由于我们无法微分，所以用差值去拟合微分，也就是代码的最后一行。

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

`GetProjection` 函数的核心思想是将空间中的点投影到车道中心线（由一系列线段组成）上，通过遍历所有线段，找到距离点最近的线段，并计算该点在该线段上的投影位置。投影的本质是利用向量点积和投影公式，将点到线段的向量分解为沿线段方向和垂直方向的分量，从而得到累计弧长s（即点在中心线上的投影距离）和横向距离lateral（即点到中心线的最短距离，带符号表示左右）。

#### 6.7 UpdateOverlaps

```cpp

void LaneInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void LaneInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr =
        map_instance.GetOverlapById(MakeMapId(overlap_id));
    if (overlap_ptr == nullptr) {
      continue;
    }
    overlaps_.emplace_back(overlap_ptr);
    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == lane_.id().id()) {
        continue;
      }
      const auto &object_map_id = MakeMapId(object_id);
      if (map_instance.GetLaneById(object_map_id) != nullptr) {
        cross_lanes_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSignalById(object_map_id) != nullptr) {
        signals_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetYieldSignById(object_map_id) != nullptr) {
        yield_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetStopSignById(object_map_id) != nullptr) {
        stop_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetCrosswalkById(object_map_id) != nullptr) {
        crosswalks_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetJunctionById(object_map_id) != nullptr) {
        junctions_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetClearAreaById(object_map_id) != nullptr) {
        clear_areas_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSpeedBumpById(object_map_id) != nullptr) {
        speed_bumps_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetParkingSpaceById(object_map_id) != nullptr) {
        parking_spaces_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetPNCJunctionById(object_map_id) != nullptr) {
        pnc_junctions_.emplace_back(overlap_ptr);
      }
    }
  }
}
```

`UpdateOverlaps`函数用于根据车道的重叠ID列表，遍历所有重叠区域，并将与当前车道相关的信号灯、让行标志、停车标志、人行横道、路口、清除区、减速带、停车位、PNC路口等对象的重叠信息分类存储到对应的成员变量中。这里是使用了前向声明，所以这里就先不对`HDMapImpl`进行介绍了。

#### 6.8 CreateKDTree

```cpp
void LaneInfo::CreateKDTree() {
  apollo::common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;

  segment_box_list_.clear();
  for (size_t id = 0; id < segments_.size(); ++id) {
    const auto &segment = segments_[id];
    segment_box_list_.emplace_back(
        apollo::common::math::AABox2d(segment.start(), segment.end()), this,
        &segment, id);
  }
  lane_segment_kdtree_.reset(new LaneSegmentKDTree(segment_box_list_, params));
}
```

`CreateKDTree`函数用于为当前车道的所有线段构建AABoxKDTree2d空间索引结构。它会遍历车道中心线的每一段，生成对应的包围盒对象（LaneSegmentBox），并根据设定的参数初始化KD树。

### 7. JunctionInfo

```cpp
class JunctionInfo {
 public:
  explicit JunctionInfo(const Junction &junction);

  const Id &id() const { return junction_.id(); }
  const Junction &junction() const { return junction_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

  const std::vector<Id> &OverlapStopSignIds() const {
    return overlap_stop_sign_ids_;
  }

 private:
  friend class HDMapImpl;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);

 private:
  const Junction &junction_;
  apollo::common::math::Polygon2d polygon_;

  std::vector<Id> overlap_stop_sign_ids_;
  std::vector<Id> overlap_ids_;
};

JunctionInfo::JunctionInfo(const Junction &junction) : junction_(junction) {
  Init();
}

void JunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void JunctionInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void JunctionInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }

      if (object.has_stop_sign_overlap_info()) {
        overlap_stop_sign_ids_.push_back(object.id());
      }
    }
  }
}
```

`JunctionInfo`类用于封装和管理地图中路口（Junction）的相关信息。
该类包含路口的ID、原始Junction对象、多边形区域，以及与路口重叠的停车标志ID集合。

### 8. SignalInfo

```cpp
class SignalInfo {
 public:
  explicit SignalInfo(const Signal &signal);

  const Id &id() const { return signal_.id(); }
  const Signal &signal() const { return signal_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const Signal &signal_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};

SignalInfo::SignalInfo(const Signal &signal) : signal_(signal) { Init(); }

void SignalInfo::Init() {
  for (const auto &stop_line : signal_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());
  std::vector<Vec2d> points;
  for (const auto &segment : segments_) {
    points.emplace_back(segment.start());
    points.emplace_back(segment.end());
  }
  CHECK_GT(points.size(), 0U);
}
```

SignalInfo类用于封装和管理地图中的信号灯信息。该类包含信号灯的ID、原始Signal对象，以及所有停止线的线段集合。在初始化时，会遍历信号灯关联的所有停止线，将其曲线转换为线段并存储。

### 9. CrosswalkInfo

```cpp
class CrosswalkInfo {
 public:
  explicit CrosswalkInfo(const Crosswalk &crosswalk);

  const Id &id() const { return crosswalk_.id(); }
  const Crosswalk &crosswalk() const { return crosswalk_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const Crosswalk &crosswalk_;
  apollo::common::math::Polygon2d polygon_;
};

CrosswalkInfo::CrosswalkInfo(const Crosswalk &crosswalk)
    : crosswalk_(crosswalk) {
  Init();
}

void CrosswalkInfo::Init() {
  polygon_ = ConvertToPolygon2d(crosswalk_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}
```

CrosswalkInfo类用于封装和管理地图中的人行横道（Crosswalk）信息。该类包含人行横道的ID、原始Crosswalk对象，以及其多边形区域。

### 10. StopSignInfo

```cpp
class StopSignInfo {
 public:
  explicit StopSignInfo(const StopSign &stop_sign);

  const Id &id() const { return stop_sign_.id(); }
  const StopSign &stop_sign() const { return stop_sign_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<Id> &OverlapLaneIds() const { return overlap_lane_ids_; }
  const std::vector<Id> &OverlapJunctionIds() const {
    return overlap_junction_ids_;
  }

 private:
  friend class HDMapImpl;
  void init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);

 private:
  const StopSign &stop_sign_;
  std::vector<apollo::common::math::LineSegment2d> segments_;

  std::vector<Id> overlap_lane_ids_;
  std::vector<Id> overlap_junction_ids_;
  std::vector<Id> overlap_ids_;
};

StopSignInfo::StopSignInfo(const StopSign &stop_sign) : stop_sign_(stop_sign) {
  init();
}

void StopSignInfo::init() {
  for (const auto &stop_line : stop_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());

  for (const auto &overlap_id : stop_sign_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void StopSignInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void StopSignInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }

      if (object.has_junction_overlap_info()) {
        overlap_junction_ids_.push_back(object.id());
      } else if (object.has_lane_overlap_info()) {
        overlap_lane_ids_.push_back(object.id());
      }
    }
  }
  if (overlap_junction_ids_.empty()) {
    AWARN << "stop sign " << id().id() << "has no overlap with any junction.";
  }
}
```

`StopSignInfo`类用于封装和管理地图中的停车标志（StopSign）信息。该类包含停车标志的ID、原始StopSign对象、所有停止线的线段集合，以及与之重叠的车道ID和路口ID集合。

### 11. YieldSignInfo

```cpp
class YieldSignInfo {
 public:
  explicit YieldSignInfo(const YieldSign &yield_sign);

  const Id &id() const { return yield_sign_.id(); }
  const YieldSign &yield_sign() const { return yield_sign_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const YieldSign &yield_sign_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};

YieldSignInfo::YieldSignInfo(const YieldSign &yield_sign)
    : yield_sign_(yield_sign) {
  Init();
}

void YieldSignInfo::Init() {
  for (const auto &stop_line : yield_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  // segments_from_curve(yield_sign_.stop_line(), &segments_);
  ACHECK(!segments_.empty());
}
```

`YieldSignInfo`用于封装地图中的让行标志（YieldSign）信息，主要负责解析和存储让行标志的停止线几何数据。

### 12. ClearAreaInfo

```cpp
class ClearAreaInfo {
 public:
  explicit ClearAreaInfo(const ClearArea &clear_area);

  const Id &id() const { return clear_area_.id(); }
  const ClearArea &clear_area() const { return clear_area_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ClearArea &clear_area_;
  apollo::common::math::Polygon2d polygon_;
};

OverlapInfo::OverlapInfo(const Overlap &overlap) : overlap_(overlap) {}

const ObjectOverlapInfo *OverlapInfo::GetObjectOverlapInfo(const Id &id) const {
  for (const auto &object : overlap_.object()) {
    if (object.id().id() == id.id()) {
      return &object;
    }
  }
  return nullptr;
}
```

### 13. SpeedBumpInfo

```cpp
class SpeedBumpInfo {
 public:
  explicit SpeedBumpInfo(const SpeedBump &speed_bump);

  const Id &id() const { return speed_bump_.id(); }
  const SpeedBump &speed_bump() const { return speed_bump_; }
  const std::vector<apollo::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const SpeedBump &speed_bump_;
  std::vector<apollo::common::math::LineSegment2d> segments_;
};
```

### 14. OverlapInfo

```cpp
class OverlapInfo {
 public:
  explicit OverlapInfo(const Overlap &overlap);

  const Id &id() const { return overlap_.id(); }
  const Overlap &overlap() const { return overlap_; }
  const ObjectOverlapInfo *GetObjectOverlapInfo(const Id &id) const;

 private:
  const Overlap &overlap_;
};
```

OverlapInfo用于封装地图中重叠区域（Overlap）的信息，支持通过ID检索与该重叠区域相关的对象，

### 15. RoadInfo

```cpp
class RoadInfo {
 public:
  explicit RoadInfo(const Road &road);
  const Id &id() const { return road_.id(); }
  const Road &road() const { return road_; }
  const std::vector<RoadSection> &sections() const { return sections_; }

  const Id &junction_id() const { return road_.junction_id(); }
  bool has_junction_id() const { return road_.has_junction_id(); }

  const std::vector<RoadBoundary> &GetBoundaries() const;

  apollo::hdmap::Road_Type type() const { return road_.type(); }

 private:
  Road road_;
  std::vector<RoadSection> sections_;
  std::vector<RoadBoundary> road_boundaries_;
};

RoadInfo::RoadInfo(const Road &road) : road_(road) {
  for (const auto &section : road_.section()) {
    sections_.push_back(section);
    road_boundaries_.push_back(section.boundary());
  }
}

const std::vector<RoadBoundary> &RoadInfo::GetBoundaries() const {
  return road_boundaries_;
}
```

RoadInfo用于封装和管理地图中的道路（Road）信息，包含道路的ID、原始Road对象、各个路段（Section）及其边界。

### 16. ParkingSpaceInfo

```cpp
class ParkingSpaceInfo {
 public:
  explicit ParkingSpaceInfo(const ParkingSpace &parkingspace);
  const Id &id() const { return parking_space_.id(); }
  const ParkingSpace &parking_space() const { return parking_space_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ParkingSpace &parking_space_;
  apollo::common::math::Polygon2d polygon_;
};


ParkingSpaceInfo::ParkingSpaceInfo(const ParkingSpace &parking_space)
    : parking_space_(parking_space) {
  Init();
}

void ParkingSpaceInfo::Init() {
  polygon_ = ConvertToPolygon2d(parking_space_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

```

ParkingSpaceInfo用于封装和管理地图中的停车位（ParkingSpace）信息，包含停车位的ID、原始ParkingSpace对象及其多边形区域。

### 17. PNCJunctionInfo

```cpp
class PNCJunctionInfo {
 public:
  explicit PNCJunctionInfo(const PNCJunction &pnc_junction);

  const Id &id() const { return junction_.id(); }
  const PNCJunction &pnc_junction() const { return junction_; }
  const apollo::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const PNCJunction &junction_;
  apollo::common::math::Polygon2d polygon_;

  std::vector<Id> overlap_ids_;
};

PNCJunctionInfo::PNCJunctionInfo(const PNCJunction &pnc_junction)
    : junction_(pnc_junction) {
  Init();
}

void PNCJunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}
```

PNCJunctionInfo用于封装和管理地图中的PNC路口（PNCJunction）信息，包含路口的ID、原始PNCJunction对象、多边形区域及其重叠ID。



Reference:

