<!--
 * @Author: LOTEAT
 * @Date: 2025-08-03 10:57:47
-->

## HDMapUtil详解

[知乎链接](https://zhuanlan.zhihu.com/p/1935727865837647323)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/map/HDMapUtil/hdmap_util.md)

本篇博客主要介绍一下apollo中HDMap的辅助工具类`HDMapUtil`。

### 1. Utility Function

#### 1.1 FindFirstExist
```cpp
std::string FindFirstExist(const std::string& dir, const std::string& files) {
  const std::vector<std::string> candidates = absl::StrSplit(files, '|');
  for (const auto& filename : candidates) {
    const std::string file_path = absl::StrCat(FLAGS_map_dir, "/", filename);
    if (cyber::common::PathExists(file_path)) {
      return file_path;
    }
  }
  AERROR << "No existing file found in " << dir << "/" << files
         << ". Fallback to first candidate as default result.";
  ACHECK(!candidates.empty()) << "Please specify at least one map.";
  return absl::StrCat(FLAGS_map_dir, "/", candidates[0]);
}
```

FindFirstExist 用于在指定目录下，根据候选文件名列表查找第一个实际存在的文件路径。如果所有候选文件都不存在，则返回第一个候选项的默认路径。

#### 1.2 MapFile
```cpp
std::string BaseMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "base_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FLAGS_test_base_map_filename.empty()
             ? FindFirstExist(FLAGS_map_dir, FLAGS_base_map_filename)
             : FindFirstExist(FLAGS_map_dir, FLAGS_test_base_map_filename);
}

std::string SimMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "sim_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_sim_map_filename);
}

std::string RoutingMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "routing_map file is not used when FLAGS_use_navigation_mode is "
             "true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_routing_map_filename);
}
```
BaseMapFile、SimMapFile 和 RoutingMapFile 分别用于获取基础地图、仿真地图和路由地图的实际文件路径，均通过 FindFirstExist 实现自动查找和容错。

这里简单介绍一下gflags。gflags 是一个用于 C++ 项目的命令行参数解析库，支持通过全局变量（如 FLAGS_xxx）灵活配置程序运行参数。在 Apollo 项目中，用于动态指定地图路径、模式开关等等。一部分的gflags设定都在`modules/common/configs/config_gflags.h`和`modules/common/adapters/adapter_gflags.h`中，还有一部分在各个模块中自行设定。

#### 1.3 CreateMap
```cpp
std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromFile(map_file_path) != 0) {
    AERROR << "Failed to load HDMap " << map_file_path;
    return nullptr;
  }
  AINFO << "Load HDMap success: " << map_file_path;
  return hdmap;
}

std::unique_ptr<HDMap> CreateMap(const MapMsg& map_msg) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromProto(map_msg.hdmap()) != 0) {
    AERROR << "Failed to load RelativeMap: "
           << map_msg.header().ShortDebugString();
    return nullptr;
  }
  return hdmap;
}
```
CreateMap 提供了两种方式创建 HDMap 实例：可以从地图文件路径加载高精地图，也可以直接从 MapMsg 消息体加载。

### 2. MapUtil
```cpp
class HDMapUtil {
 public:
  // Get default base map from the file specified by global flags.
  // Return nullptr if failed to load.
  static const HDMap* BaseMapPtr();
  static const HDMap* BaseMapPtr(const relative_map::MapMsg& map_msg);
  // Guarantee to return a valid base_map, or else raise fatal error.
  static const HDMap& BaseMap();

  // Get default sim_map from the file specified by global flags.
  // Return nullptr if failed to load.
  static const HDMap* SimMapPtr();

  // Guarantee to return a valid sim_map, or else raise fatal error.
  static const HDMap& SimMap();

  // Reload maps from the file specified by global flags.
  static bool ReloadMaps();

  static bool ReloadBaseMap();

 private:
  HDMapUtil() = delete;

  static std::unique_ptr<HDMap> base_map_;
  static uint64_t base_map_seq_;
  static std::mutex base_map_mutex_;

  static std::unique_ptr<HDMap> sim_map_;
  static std::mutex sim_map_mutex_;
};
```

HDMapUtil 是高精地图的全局辅助工具类，提供基础地图和仿真地图的统一加载、获取、重载等接口，支持多线程安全访问。


#### 2.1 BaseMap
```cpp
const HDMap* HDMapUtil::BaseMapPtr(const MapMsg& map_msg) {
  std::lock_guard<std::mutex> lock(base_map_mutex_);
  if (base_map_ != nullptr &&
      base_map_seq_ == map_msg.header().sequence_num()) {
    // avoid re-create map in the same cycle.
    return base_map_.get();
  } else {
    base_map_ = CreateMap(map_msg);
    base_map_seq_ = map_msg.header().sequence_num();
  }
  return base_map_.get();
}

const HDMap* HDMapUtil::BaseMapPtr() {
  // TODO(all) Those logics should be removed to planning
  /*if (FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    auto* relative_map = AdapterManager::GetRelativeMap();
    if (!relative_map) {
      AERROR << "RelativeMap adapter is not registered";
      return nullptr;
    }
    if (relative_map->Empty()) {
      AERROR << "RelativeMap is empty";
      return nullptr;
    }
    const auto& latest = relative_map->GetLatestObserved();
    if (base_map_ != nullptr &&
        base_map_seq_ == latest.header().sequence_num()) {
      // avoid re-create map in the same cycle.
      return base_map_.get();
    } else {
      base_map_ = CreateMap(latest);
      base_map_seq_ = latest.header().sequence_num();
    }
  } else*/
  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      base_map_ = CreateMap(BaseMapFile());
    }
  }
  return base_map_.get();
}

const HDMap& HDMapUtil::BaseMap() { return *CHECK_NOTNULL(BaseMapPtr()); }
```

BaseMapPtr 用于获取当前的基础高精地图指针，支持根据 MapMsg 消息体自动判断是否需要重新加载地图，保证同一周期内不会重复创建，提升效率和线程安全。

#### 2.2 SimMap
```cpp
const HDMap* HDMapUtil::SimMapPtr() {
  if (FLAGS_use_navigation_mode) {
    return BaseMapPtr();
  } else if (sim_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    if (sim_map_ == nullptr) {  // Double check.
      sim_map_ = CreateMap(SimMapFile());
    }
  }
  return sim_map_.get();
}

const HDMap& HDMapUtil::SimMap() { return *CHECK_NOTNULL(SimMapPtr()); }

```
SimMap与BaseMap同理。

#### 2.3 ReloadMaps
```cpp
bool HDMapUtil::ReloadMaps() {
  {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    base_map_ = CreateMap(BaseMapFile());
  }
  {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    sim_map_ = CreateMap(SimMapFile());
  }
  return base_map_ != nullptr && sim_map_ != nullptr;
}

bool HDMapUtil::ReloadBaseMap() {
  {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    base_map_ = CreateMap(BaseMapFile());
  }
  return base_map_ != nullptr;
}
```

ReloadBaseMap 用于重新加载基础高精地图，确保在地图文件发生变更或需要刷新时能够及时更新内存中的地图数据。
