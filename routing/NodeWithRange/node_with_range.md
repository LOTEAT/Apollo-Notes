<!--
 * @Author: LOTEAT
 * @Date: 2025-08-13 09:26:21
-->


## NodeWithRange详解

[知乎链接](https://zhuanlan.zhihu.com/p/1938986091551101758)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/routing/NodeWithRange/node_with_range.md)


### 1. Defination
```cpp
class NodeWithRange : public NodeSRange {
 public:
  NodeWithRange(const NodeWithRange& other) = default;
  NodeWithRange(const TopoNode* node, double start_s, double end_s);
  NodeWithRange(const TopoNode* node, const NodeSRange& range);
  virtual ~NodeWithRange();
  bool operator<(const NodeWithRange& other) const;

  const TopoNode* GetTopoNode() const;
  bool IsVirtual() const;
  const std::string& RoadId() const;
  const std::string& LaneId() const;
  double FullLength() const;

 private:
  const TopoNode* topo_node_ = nullptr;
};
```

`NodeWithRange` 类通过继承 `NodeSRange` 类实现了范围信息与拓扑节点的结合。与基类 `NodeSRange` 仅包含起始和结束坐标的范围描述不同，`NodeWithRange` 通过添加 `topo_node_` 成员变量将拓扑节点与空间范围信息关联起来。


### 2. 构造函数
```cpp
NodeWithRange(const NodeWithRange& other) = default;

NodeWithRange::NodeWithRange(const TopoNode* node, const NodeSRange& range)
    : NodeSRange(range), topo_node_(node) {}

NodeWithRange::NodeWithRange(const TopoNode* node, double start_s, double end_s)
    : NodeSRange(start_s, end_s), topo_node_(node) {}
```

`NodeWithRange` 类提供了三种构造方式。第一个是默认的拷贝构造函数。第二个构造函数接受一个 `TopoNode` 指针和一个 `NodeSRange` 对象，通过调用父类 `NodeSRange`构造函数，同时将节点指针赋值给 `topo_node_` 成员变量。第三个构造函数则直接接受拓扑节点指针和起始结束坐标参数，通过调用基类的构造函数完成范围初始化。
