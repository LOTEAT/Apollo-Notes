<!--
 * @Author: LOTEAT
 * @Date: 2025-08-25 16:02:41
-->
## Topo Creator详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/TopoCreator/topo_creator.md)

### 1. Code
```cpp
#include "cyber/common/file.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/graph_creator.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::routing::RoutingConfig routing_conf;

  ACHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_routing_conf_file,
                                                 &routing_conf))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  const auto base_map = apollo::hdmap::BaseMapFile();
  const auto routing_map = apollo::hdmap::RoutingMapFile();

  apollo::routing::GraphCreator creator(base_map, routing_map, routing_conf);
  ACHECK(creator.Create()) << "Create routing topo failed!";

  AINFO << "Create routing topo successfully from " << base_map << " to "
        << routing_map;
  return 0;
}
```

这段代码是 Apollo 自动驾驶系统中的拓扑图创建工具的程序，负责将高精地图转换为路径规划所需的拓扑图结构。程序首先初始化 Google 日志系统和命令行参数解析，然后从配置文件中加载路径规划的相关配置参数。接下来获取基础地图文件和路径规划地图文件的路径，创建 `GraphCreator` 对象并调用其 `Create()` 方法来执行实际的拓扑图生成工作。

### 2. 使用示例
在这段代码中，有两个命令行参数。一个是`flagfile`，这个参数固定，值可以设置为`modules/routing/conf/routing.conf`。一个是`map_dir`，这个是你想要转的地图所在的文件夹。输入命令为`/apollo/bazel-bin/modules/routing/topo_creator --flagfile=modules/routing/conf/routing.conf --map_dir=xxx`。