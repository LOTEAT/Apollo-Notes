<!--
 * @Author: LOTEAT
 * @Date: 2025-08-06 09:13:16
-->
## ObjectPool详解

[知乎链接](https://zhuanlan.zhihu.com/p/1936016024702026293)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/cyber/ObjectPool/object_pool.md)

### 1. Defination
```cpp
template <typename T>
class ObjectPool : public std::enable_shared_from_this<ObjectPool<T>> {
 public:
  using InitFunc = std::function<void(T *)>;
  using ObjectPoolPtr = std::shared_ptr<ObjectPool<T>>;

  template <typename... Args>
  explicit ObjectPool(uint32_t num_objects, Args &&... args);

  template <typename... Args>
  ObjectPool(uint32_t num_objects, InitFunc f, Args &&... args);

  virtual ~ObjectPool();

  std::shared_ptr<T> GetObject();

 private:
  struct Node {
    T object;
    Node *next;
  };

  ObjectPool(ObjectPool &) = delete;
  ObjectPool &operator=(ObjectPool &) = delete;
  void ReleaseObject(T *);

  uint32_t num_objects_ = 0;
  char *object_arena_ = nullptr;
  Node *free_head_ = nullptr;
};
```

`ObjectPool` 是 Apollo 框架中一个内存池实现，专门用于管理固定数量的同类型对象，以解决频繁创建和销毁对象所带来的性能开销和内存碎片问题。该类继承自 `std::enable_shared_from_this<ObjectPool<T>>`，使得对象池自身可以安全地生成指向自己的共享指针。

`ObjectPool` 通过预分配内存和链表管理来进行对象管理。`object_arena_` 是一个连续的内存区域，在对象池创建时一次性分配足够容纳所有对象的内存空间，避免了运行时的动态内存分配开销。每个对象被包装在 `Node` 结构中，该结构不仅包含实际的对象实例，还包含指向下一个空闲节点的指针，从而构成一个单向链表来管理空闲对象。`free_head_` 指针始终指向当前链表的头部。

### 2. ObjectPool
```cpp
template <typename T>
template <typename... Args>
ObjectPool<T>::ObjectPool(uint32_t num_objects, Args &&... args)
    : num_objects_(num_objects) {
  const size_t size = sizeof(Node);
  object_arena_ = static_cast<char *>(std::calloc(num_objects_, size));
  if (object_arena_ == nullptr) {
    throw std::bad_alloc();
  }

  FOR_EACH(i, 0, num_objects_) {
    T *obj = new (object_arena_ + i * size) T(std::forward<Args>(args)...);
    reinterpret_cast<Node *>(obj)->next = free_head_;
    free_head_ = reinterpret_cast<Node *>(obj);
  }
}

template <typename T>
template <typename... Args>
ObjectPool<T>::ObjectPool(uint32_t num_objects, InitFunc f, Args &&... args)
    : num_objects_(num_objects) {
  const size_t size = sizeof(Node);
  object_arena_ = static_cast<char *>(std::calloc(num_objects_, size));
  if (object_arena_ == nullptr) {
    throw std::bad_alloc();
  }

  FOR_EACH(i, 0, num_objects_) {
    T *obj = new (object_arena_ + i * size) T(std::forward<Args>(args)...);
    f(obj);
    reinterpret_cast<Node *>(obj)->next = free_head_;
    free_head_ = reinterpret_cast<Node *>(obj);
  }
}
```

`ObjectPool` 提供了两个构造函数重载，分别支持不同的对象初始化需求。第一个构造函数 `ObjectPool(uint32_t num_objects, Args &&... args)` 是基础版本，它接受对象数量和可变参数模板，这些参数会被完美转发（forward函数）给每个对象的构造函数。第二个构造函数 `ObjectPool(uint32_t num_objects, InitFunc f, Args &&... args)` 在基础版本的基础上增加了自定义初始化函数，允许在对象构造完成后进行额外的初始化操作。

两个构造函数都遵循相同的内存分配策略，首先通过 `sizeof(Node)` 计算每个节点所需的内存大小，然后使用 `std::calloc` 分配连续的内存区域来容纳所有对象。如果内存分配失败，构造函数会立即抛出 `std::bad_alloc` 异常。

在对象初始化阶段，构造函数使用了 placement new 技术，即 `new (object_arena_ + i * size) T(std::forward<Args>(args)...)`，这种技术允许在预分配的内存位置上直接构造对象，避免了额外的内存分配开销。`std::forward<Args>(args)...` 确保了参数的完美转发，保持了参数的值类别（左值或右值）。对于带有初始化函数的构造函数版本，在对象构造完成后会立即调用 `f(obj)` 对对象进行额外的初始化处理。

构造函数的最后阶段是建立空闲对象链表。每个新构造的对象都会被强制类型转换为 `Node*` 类型，然后设置其 `next` 指针指向当前的 `free_head_`，接着将 `free_head_` 更新为当前对象的地址。这种操作实际上是在链表头部插入新节点，由于是在构造阶段进行的，插入顺序与内存地址顺序相反，但这并不影响对象池的功能，因为空闲链表只需要能够提供可用对象即可。其构造完成后，结构如下：
```shell
free_head_ --> obj(i)
               |
               v
           obj(i-1)
               |
               v
             ...
               |
               v
           obj(0)
               |
               v
             nullptr
```

### 3. ~ObjectPool
```cpp
template <typename T>
ObjectPool<T>::~ObjectPool() {
  if (object_arena_ != nullptr) {
    const size_t size = sizeof(Node);
    FOR_EACH(i, 0, num_objects_) {
      reinterpret_cast<Node *>(object_arena_ + i * size)->object.~T();
    }
    std::free(object_arena_);
  }
}
```

析构函数首先检查 `object_arena_` 是否为非空指针。随后，函数遍历内存池中的每个对象位置，通过 `reinterpret_cast<Node *>(object_arena_ + i * size)->object.~T()` 显式调用每个对象的析构函数，这是因为使用 placement new 构造的对象不会被自动销毁，必须手动调用析构函数来释放对象持有的资源。最后，通过 `std::free(object_arena_)` 释放整个内存池的原始内存空间。

### 4. ReleaseObject & GetObject
```cpp
template <typename T>
void ObjectPool<T>::ReleaseObject(T *object) {
  if (cyber_unlikely(object == nullptr)) {
    return;
  }

  reinterpret_cast<Node *>(object)->next = free_head_;
  free_head_ = reinterpret_cast<Node *>(object);
}
```

`ReleaseObject` 函数是对象池回收机制，负责将使用完毕的对象重新归还到空闲对象链表中，以便后续重复使用。函数首先通过 `cyber_unlikely(object == nullptr)` 进行空指针检查，使用了分支预测优化宏 `cyber_unlikely` 来提示编译器这种情况发生的概率很低，从而优化正常路径的性能。当传入有效对象指针时，函数将对象强制转换为 `Node*` 类型，然后设置其 `next` 指针指向当前的 `free_head_`，接着将 `free_head_` 更新为当前对象的地址，这实际上是在空闲链表的头部插入节点的操作。

`ReleaseObject`最好与`GetObject`放在一起理解。

```cpp
template <typename T>
std::shared_ptr<T> ObjectPool<T>::GetObject() {
  if (cyber_unlikely(free_head_ == nullptr)) {
    return nullptr;
  }

  auto self = this->shared_from_this();
  auto obj =
      std::shared_ptr<T>(reinterpret_cast<T *>(free_head_),
                         [self](T *object) { self->ReleaseObject(object); });
  free_head_ = free_head_->next;
  return obj;
}
```

`GetObject` 函数是对象池的获取接口。函数首先通过 `cyber_unlikely(free_head_ == nullptr)` 检查是否还有可用对象，然后创建一个带有自定义删除器的 `shared_ptr<T>`，该删除器捕获了对象池的引用并在对象生命周期结束时调用 `ReleaseObject` 而不是 `delete`。一旦对象销毁，那么就会调用`ReleaseObject`，把这个对象重新放回链表头部。