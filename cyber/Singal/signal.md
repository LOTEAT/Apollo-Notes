<!--
 * @Author: LOTEAT
 * @Date: 2025-08-14 15:34:52
-->
## Signal详解

[知乎链接]()

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/cyber/Singal/signal.md)

Signal由三部分组成，信号（Signal）、连接（Connection）以及槽函数（Slot）。在一个信号触发时，由连接关系确定触发后所调用的函数，也就是槽函数。

### 1. Signal
```cpp
template <typename... Args>
class Slot;

template <typename... Args>
class Connection;
```
首先，前向定义`Slot`和`Connection`，都使用变参模板。

```cpp
template <typename... Args>
class Signal {
public:
  using Callback = std::function<void(Args...)>;
  using SlotPtr = std::shared_ptr<Slot<Args...>>;
  using SlotList = std::list<SlotPtr>;
  using ConnectionType = Connection<Args...>;
```

这几个 `using` 声明定义了 Signal 类中使用的核心类型别名：`Callback` 是一个函数对象类型，用于表示接收可变参数并返回 void 的回调函数；`SlotPtr` 是指向 Slot 对象的智能指针，用于管理槽函数的生命周期；`SlotList` 是存储多个槽函数指针的链表容器；`ConnectionType` 是连接类型的别名，用于表示信号与槽函数之间的连接关系。

```cpp
  Signal() {}
  virtual ~Signal() { DisconnectAllSlots(); }
```
构造函数和析构函数，析构函数会断掉所有的槽函数连接。

```cpp
  void operator()(Args... args) {
    SlotList local;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& slot : slots_) {
        local.emplace_back(slot);
      }
    }

    if (!local.empty()) {
      for (auto& slot : local) {
        (*slot)(args...);
      }
    }

    ClearDisconnectedSlots();
  }
```

函数调用运算符重载 `operator()` 是信号触发的实现，它接收可变参数并将信号传播给所有连接的槽函数。该函数首先创建一个本地槽函数列表副本，然后在互斥锁保护下将当前所有槽函数复制到本地列表中。如果直接在原始槽函数列表上进行遍历调用，当某个槽函数在执行过程中修改了槽函数列表（比如断开连接或添加新连接），就可能导致迭代器失效或竞态条件。通过创建本地副本，可以确保信号触发过程中槽函数列表的稳定性，同时释放互斥锁以避免长时间锁定，最后调用 `ClearDisconnectedSlots()` 清理已断开连接的槽函数。

```cpp
  ConnectionType Connect(const Callback& cb) {
    auto slot = std::make_shared<Slot<Args...>>(cb);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slots_.emplace_back(slot);
    }

    return ConnectionType(slot, this);
  }
```
添加一个新的槽函数。
```cpp
  bool Disconnect(const ConnectionType& conn) {
    bool find = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& slot : slots_) {
        if (conn.HasSlot(slot)) {
          find = true;
          slot->Disconnect();
        }
      }
    }

    if (find) {
      ClearDisconnectedSlots();
    }
    return find;
  }
```
断掉某个槽函数的连接，并通过调用`ClearDisconnectedSlots`来清除不连接的槽函数。
```cpp
  void DisconnectAllSlots() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slot : slots_) {
      slot->Disconnect();
    }
    slots_.clear();
  }

```
把所有的槽函数都断连。
```cpp
 private:
  Signal(const Signal&) = delete;
  Signal& operator=(const Signal&) = delete;

  void ClearDisconnectedSlots() {
    std::lock_guard<std::mutex> lock(mutex_);
    slots_.erase(
        std::remove_if(slots_.begin(), slots_.end(),
                       [](const SlotPtr& slot) { return !slot->connected(); }),
        slots_.end());
  }

  SlotList slots_;
  std::mutex mutex_;
};
```
private成员和变量。


### 2. Connection
```cpp
template <typename... Args>
class Connection {
 public:
  using SlotPtr = std::shared_ptr<Slot<Args...>>;
  using SignalPtr = Signal<Args...>*;

  Connection() : slot_(nullptr), signal_(nullptr) {}
  Connection(const SlotPtr& slot, const SignalPtr& signal)
      : slot_(slot), signal_(signal) {}
  virtual ~Connection() {
    slot_ = nullptr;
    signal_ = nullptr;
  }

  Connection& operator=(const Connection& another) {
    if (this != &another) {
      this->slot_ = another.slot_;
      this->signal_ = another.signal_;
    }
    return *this;
  }

  bool HasSlot(const SlotPtr& slot) const {
    if (slot != nullptr && slot_ != nullptr) {
      return slot_.get() == slot.get();
    }
    return false;
  }

  bool IsConnected() const {
    if (slot_) {
      return slot_->connected();
    }
    return false;
  }

  bool Disconnect() {
    if (signal_ && slot_) {
      return signal_->Disconnect(*this);
    }
    return false;
  }

 private:
  SlotPtr slot_;
  SignalPtr signal_;
};
```

`Connection` 类是信号槽机制中的连接管理器，负责维护信号与槽函数之间的关联关系。该类通过智能指针 `slot_` 和原始指针 `signal_` 分别持有槽函数和信号的引用，提供了完整的连接管理功能：`HasSlot` 方法用于检查连接是否包含特定的槽函数，通过比较智能指针的原始指针地址来判断；`IsConnected` 方法检查连接的有效性，即槽函数是否仍然处于连接状态；`Disconnect` 方法提供主动断开连接的能力，它会调用信号对象的 `Disconnect` 方法来移除该连接。

### 3. Slot
```cpp
template <typename... Args>
class Slot {
 public:
  using Callback = std::function<void(Args...)>;
  Slot(const Slot& another)
      : cb_(another.cb_), connected_(another.connected_) {}
  explicit Slot(const Callback& cb, bool connected = true)
      : cb_(cb), connected_(connected) {}
  virtual ~Slot() {}

  void operator()(Args... args) {
    if (connected_ && cb_) {
      cb_(args...);
    }
  }

  void Disconnect() { connected_ = false; }
  bool connected() const { return connected_; }

 private:
  Callback cb_;
  bool connected_ = true;
};
```

`Slot` 类是信号槽机制中的槽函数封装器，负责存储和执行具体的回调函数。该类通过 `cb_` 成员变量保存回调函数对象，并使用 `connected_` 布尔标志来标识槽函数的连接状态。槽函数的核心功能通过函数调用运算符 `operator()` 实现，它在槽函数处于连接状态且回调函数有效时执行实际的回调操作，将接收到的参数转发给内部的回调函数。`Disconnect` 方法通过将 `connected_` 标志设置为 false 来断开槽函数连接，而 `connected` 方法则用于查询当前连接状态。
