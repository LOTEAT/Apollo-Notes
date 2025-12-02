<!--
 * @Author: LOTEAT
 * @Date: 2025-08-04 16:02:57
-->

## RoutingComponent详解

[知乎链接]()

[Github]()

本篇博客主要介绍一下apollo中的`RoutingComponent`。Component 是 Cyber RT 框架（Apollo 的通信框架）中最核心的构建模块之一，类似于一个功能单元或进程内的可独立运行实体，用于处理感知、规划、控制等各个子系统中的具体任务。

一个 Component 一般包含以下几个关键部分：

-   消息订阅：用于接收来自其他 Component 的数据，比如雷达点云、摄像头图像、定位信息等。

-   消息处理逻辑：处理接收到的数据，完成自己的功能（如目标检测、路径规划等）。

-   消息发布：将处理结果发送出去，供后续模块使用。

-   配置与调度：通过 .dag 文件进行调度和依赖设置，决定何时启动、与哪些组件连接。

如果你学习过ROS，应该会对cyber框架有一个比较熟悉的感觉，cyber框架实际上正是为了替代ROS中的通信协议而创建的。这主要是因为，在机器人领域中，通信延迟重要但是并非最重要的一环。自动驾驶涉及到道路安全，其实时性要比机器人领域更为重要。cyber框架的优势之一就是为了解决进程间通信开销大这个问题的。在Apollo早期，其实也是使用ROS作为通信协议栈，但是从3.0之后，就把ROS替换为了cyber。

我原本想先介绍cyber框架，但是短时间内又无法完全讲解清楚，所以从这里开始准备对cyber和routing进行并行讲解。而至于这里的Component类，可以先理清楚其用法，有一个大致轮廓，暂时先忽略其具体实现。在cyber模块中会对Component进行一个更加详细的讲解。

### 1. RoutingComponent

```cpp
class RoutingComponent final
    : public ::apollo::cyber::Component<routing::RoutingRequest> {
 public:
  RoutingComponent() = default;
  ~RoutingComponent() = default;

 public:
  bool Init() override;
  bool Proc(
      const std::shared_ptr<routing::RoutingRequest>& request) override;

 private:
  std::shared_ptr<::apollo::cyber::Writer<routing::RoutingResponse>>
      response_writer_ = nullptr;
  std::shared_ptr<::apollo::cyber::Writer<routing::RoutingResponse>>
      response_history_writer_ = nullptr;
  Routing routing_;
  std::shared_ptr<routing::RoutingResponse> response_ = nullptr;
  std::unique_ptr<::apollo::cyber::Timer> timer_;
  std::mutex mutex_;
};

CYBER_REGISTER_COMPONENT(RoutingComponent)
```

`RoutingComponent` 是 Apollo 路径导航系统在 Cyber RT 框架中的核心组件实现，它继承自 `cyber::Component<routing::RoutingRequest>`，专门负责处理路径规划请求并生成全局导航路径。作为一个标准的 Cyber RT 组件，`RoutingComponent` 遵循了组件化设计的基本原则，通过模板参数指定了输入消息类型为 `RoutingRequest`，这意味着该组件会自动订阅和处理来自其他模块的路径导航请求。

该组件的核心接口包括 `Init()` 和 `Proc()` 两个关键方法。`Init()` 方法负责组件的初始化工作，包括创建消息发布器、加载配置参数等准备工作。`Proc()` 方法是组件的主要处理逻辑，它接收 `RoutingRequest` 消息作为输入，调用底层的路径规划算法进行路径计算，并将结果封装为 `RoutingResponse` 消息发布出去。

最后，通过 `CYBER_REGISTER_COMPONENT` 宏的注册，该组件可以被 Cyber RT 框架自动发现和管理，支持动态加载、配置管理和生命周期控制。

### 2. DAG config

在介绍其具体实现之前，我们先来简单看一下其DAG配置文件。

```json
# Define all coms in DAG streaming.
module_config {
    module_library : "modules/routing/librouting_component.so"
    components {
        class_name : "RoutingComponent"
        config {
            name : "routing"
            config_file_path: "/apollo/modules/routing/conf/routing_config.pb.txt"
            flag_file_path: "/apollo/modules/routing/conf/routing.conf"
            readers: [
                {
                    channel: "/apollo/raw_routing_request"
                    qos_profile: {
                        depth : 10
                    }
                }
            ]
        }
    }
}

```

这个 DAG 配置文件定义了 `RoutingComponent` 在 Cyber RT 框架中的运行环境和参数设置，是组件部署和运行的重要配置文件。配置文件通过 `module_library` 指定了组件所在的动态链接库路径，确保 Cyber RT 能够正确加载路径规划组件。这个路径是基于bazel的BUILD文件，接下来的讲解不会涉及bazel编译的知识，还需要大家自行寻找资料了解bazel。

`class_name` 明确指定了要实例化的组件类名，与代码中通过 `CYBER_REGISTER_COMPONENT` 注册的组件对应；`config_file_path` 和 `flag_file_path` 分别指向了组件的业务配置文件和启动参数文件，为路径导航算法提供了必要的配置参数，如地图文件路径、算法参数等。其中`config_file_path`是一个protobuf配置文件，flag_file_path则是gflags的配置文件。例如，这里的`config_file_path`为`/apollo/modules/routing/conf/routing_config.pb.txt`，这里的路径是在docker里的路径。其对应的配置文件为：

```txt
base_speed: 4.167
left_turn_penalty: 50.0
right_turn_penalty: 20.0
uturn_penalty: 100.0
change_penalty: 500.0
base_changing_length: 50.0
topic_config {
  routing_response_topic: "/apollo/raw_routing_response"
  routing_response_history_topic: "/apollo/raw_rrouting_response_history"
}
```

这里其实就是配置了一些导航参数以及话题名称，原始的protobuf文件如下：

```protobuf
syntax = "proto2";

package apollo.routing;

message TopicConfig {
  optional string routing_response_topic = 1;
  optional string routing_response_history_topic = 2;
}

message RoutingConfig {
  optional double base_speed = 1;  // base speed for node creator [m/s]
  optional double left_turn_penalty =
      2;  // left turn penalty for node creator [m]
  optional double right_turn_penalty =
      3;                              // right turn penalty for node creator [m]
  optional double uturn_penalty = 4;  // left turn penalty for node creator [m]
  optional double change_penalty = 5;  // change penalty for edge creator [m]
  optional double base_changing_length =
      6;  // base change length penalty for edge creator [m]
  optional TopicConfig topic_config = 7;
}
```

对比一下，这个`config_file_path`已经就很好理解了。

而针对gflags参数，有时候我们想要修改gflags参数，但是又不想修改代码，一种实现方式就是通过`flag_file_path`。

```txt
--flagfile=/apollo/modules/common/data/global_flagfile.txt
--routing_conf_file=/apollo/modules/routing/conf/routing_config.pb.txt

--use_road_id=false
--min_length_for_lane_change=1.0
--enable_change_lane_in_result
```

通过这种方式就可以修改gflags参数。

`readers` 配置段定义了组件的输入通道，这里指定组件订阅 `/apollo/raw_routing_request` 通道来接收导航规划请求，`qos_profile` 中的 `depth` 参数设置了消息队列的深度为 10，确保在高负载情况下不会丢失重要的路径规划请求。

### 3. launch file

```xml
<cyber>
    <module>
        <name>routing</name>
        <dag_conf>/apollo/modules/routing/dag/routing.dag</dag_conf>
        <process_name></process_name>
    </module>
</cyber>

```

通常我们启动Component有两种方法，一种是通过mainboard启动，一种是通过launch file启动，相较而言我更喜欢launch file，这种启动方式比较简单，输入命令就是`cyber start xxx.launch`。

### 4. Init

```cpp
bool RoutingComponent::Init() {
  RoutingConfig routing_conf;
  ACHECK(cyber::ComponentBase::GetProtoConfig(&routing_conf))
      << "Unable to load routing conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";

  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(routing_conf.topic_config().routing_response_topic());
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  response_writer_ = node_->CreateWriter<routing::RoutingResponse>(attr);

  apollo::cyber::proto::RoleAttributes attr_history;
  attr_history.set_channel_name(
      routing_conf.topic_config().routing_response_history_topic());
  auto qos_history = attr_history.mutable_qos_profile();
  qos_history->set_history(
      apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos_history->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos_history->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);

  response_history_writer_ =
      node_->CreateWriter<routing::RoutingResponse>(attr_history);
  std::weak_ptr<RoutingComponent> self =
      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
  timer_.reset(new ::apollo::cyber::Timer(
      FLAGS_routing_response_history_interval_ms,
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          std::lock_guard<std::mutex> guard(this->mutex_);
          if (this->response_ != nullptr) {
            auto timestamp = apollo::cyber::Clock::NowInSeconds();
            response_->mutable_header()->set_timestamp_sec(timestamp);
            this->response_history_writer_->Write(*response_);
          }
        }
      },
      false));
  timer_->Start();

  return routing_.Init().ok() && routing_.Start().ok();
}
```

`Init` 函数是 `RoutingComponent` 组件初始化的核心方法，负责完成组件运行前的所有准备工作。函数首先通过 `cyber::ComponentBase::GetProtoConfig(&routing_conf)` 加载配置文件，这里使用了 Apollo 框架提供的配置管理机制，能够自动解析 protobuf 格式的配置文件并填充到 `RoutingConfig` 对象中。

接下来，函数创建了两个消息发布器来处理不同的输出需求。第一个发布器 `response_writer_` 用于发布实时的导航规划结果，通过 `routing_conf.topic_config().routing_response_topic()` 获取配置的话题名称。在创建发布器时，函数详细配置了 QoS（Quality of Service）参数：`HISTORY_KEEP_LAST` 表示只保留最新的消息，`RELIABILITY_RELIABLE` 确保消息的可靠传输，`DURABILITY_TRANSIENT_LOCAL` 保证新订阅者能够接收到最近的消息。

类似地，函数还创建了第二个发布器 `response_history_writer_`，专门用于发布历史路径规划结果，其 QoS 配置与第一个发布器相同。这种设计使得系统能够同时提供实时路径信息和历史记录，实时信息供其他模块立即使用，历史记录则用于调试、监控和数据分析。

函数的一个重要特性是创建了定时器机制来定期发布历史信息。通过 `std::weak_ptr<RoutingComponent> self = std::dynamic_pointer_cast<RoutingComponent>(shared_from_this())` 创建了一个弱引用。定时器的回调函数使用了 lambda 表达式，定时器的间隔时间由 `FLAGS_routing_response_history_interval_ms` 标志位控制，提供了运行时的可配置性。

最后，函数调用 `routing_.Init().ok() && routing_.Start().ok()` 来初始化和启动底层的路径规划算法模块。这种链式调用确保了只有当路径规划模块成功初始化和启动后，整个组件才算初始化成功。

### 5. Proc

```cpp
bool RoutingComponent::Proc(
    const std::shared_ptr<routing::RoutingRequest>& request) {
  auto response = std::make_shared<routing::RoutingResponse>();
  if (!routing_.Process(request, response.get())) {
    return false;
  }
  common::util::FillHeader(node_->Name(), response.get());
  response_writer_->Write(response);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    response_ = std::move(response);
  }
  return true;
}
```

`Proc` 函数是 `RoutingComponent` 的核心处理方法，负责接收导航规划请求并生成相应的路径响应，体现了 Cyber RT 组件处理消息的标准模式。函数首先创建一个 `RoutingResponse` 智能指针对象来存储处理结果，然后调用底层路径规划算法 `routing_.Process(request, response.get())` 进行实际的路径计算，这个调用将输入的路径请求转换为可行驶的全局路径。如果路径规划失败，函数立即返回 `false`，确保了错误情况的快速响应。成功计算出路径后，函数通过 `common::util::FillHeader(node_->Name(), response.get())` 为响应消息填充标准的消息头信息，包括时间戳、序列号和发送者信息等元数据。接下来，函数使用 `response_writer_->Write(response)` 立即发布路径规划结果，确保下游模块能够及时接收到最新的路径信息。最后，在互斥锁的保护下，函数通过 `std::move` 将响应对象保存到成员变量 `response_` 中。

目前，如果是单纯要学会使用Component，看完这篇博客其实也应该学得七七八八了。但是要是想真正掌握Component的设计思路和原理，那还需要进一步深入学习 Cyber RT 框架的底层实现。

”路曼曼其修远兮，吾将上下而求索“。



Reference:

