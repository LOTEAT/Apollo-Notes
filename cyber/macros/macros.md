<!--
 * @Author: LOTEAT
 * @Date: 2025-08-04 16:27:32
-->

## Macros详解

[知乎链接](https://zhuanlan.zhihu.com/p/1936016024702026293)

[Github](https://github.com/LOTEAT/Apollo-Notes/blob/master/cyber/macros/macros.md)

### 1. 分支优化
```cpp
#if __GNUC__ >= 3
#define cyber_likely(x) (__builtin_expect((x), 1))
#define cyber_unlikely(x) (__builtin_expect((x), 0))
#else
#define cyber_likely(x) (x)
#define cyber_unlikely(x) (x)
#endif
```

这两个宏定义实现了基于编译器内建函数的分支预测优化，是现代高性能C++代码中常用的性能优化技术。`__builtin_expect` 是GCC编译器提供的内建函数，用于向编译器提供分支预测的提示信息，帮助编译器生成更优化的机器码。当使用 `cyber_likely(x)` 时，告诉编译器条件 `x` 很可能为真（概率接近1），编译器会将对应的代码路径优化为"热路径"，减少分支跳转的开销；相反，`cyber_unlikely(x)` 表示条件 `x` 很可能为假（概率接近0），编译器会将对应代码优化为"冷路径"。这种优化在CPU的分支预测器工作时特别有效，能够减少流水线停顿和分支误预测的性能损失。对于不支持该内建函数的编译器（GCC版本小于3），宏定义退化为原始表达式，保证了代码的兼容性。

### 2. DEFINE_TYPE_TRAIT
```cpp
#define CACHELINE_SIZE 64

#define DEFINE_TYPE_TRAIT(name, func)                     \
  template <typename T>                                   \
  struct name {                                           \
    template <typename Class>                             \
    static constexpr bool Test(decltype(&Class::func)*) { \
      return true;                                        \
    }                                                     \
    template <typename>                                   \
    static constexpr bool Test(...) {                     \
      return false;                                       \
    }                                                     \
                                                          \
    static constexpr bool value = Test<T>(nullptr);       \
  };                                                      \
                                                          \
  template <typename T>                                   \
  constexpr bool name<T>::value;
```

`DEFINE_TYPE_TRAIT` 是一个模板元编程宏，用于在编译期检测类型是否具有特定的成员函数或属性，这是现代C++中SFINAE（Substitution Failure Is Not An Error）技术的应用。该宏通过创建一个类型特征结构体来实现编译期类型检测，其核心机制基于函数重载解析和模板参数推导。

这个宏的核心作用是，用于判断`T`这个类中是否存在`func`这个函数或者属性。宏定义中包含两个重载的 `Test` 函数模板：第一个版本尝试获取指定成员函数的指针类型，如果类型 `T` 确实具有名为 `func` 的成员函数，则模板参数推导成功，该版本被选中并返回 `true`；第二个版本使用可变参数模板作为兜底选项，当第一个版本的SFINAE检测失败时，编译器会选择这个版本并返回 `false`。通过向 `Test` 函数传递 `nullptr` 参数，编译器在编译期就能确定使用哪个重载版本，从而得出类型检测的结果。

要注意的是`static constexpr bool Test(decltype(&Class::func)*)`，如果不存在`func`是不会报错的。这就是SFINAE。另外`Class`是struct内的模板，value的值是`Test<T>(nullptr)`，不要认为这个宏是用来判断Class是否有func。

### 3. cpu_relax
```cpp
inline void cpu_relax() {
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#else
  asm volatile("rep; nop" ::: "memory");
#endif
}
```

`cpu_relax` 函数是一个轻量级的CPU放松指令，主要用于自旋锁和忙等待场景中的性能优化。该函数通过内联汇编执行特定的CPU指令来暂时让出CPU资源，提高多线程环境下的系统整体性能。对于ARM64架构（`__aarch64__`），函数执行 `yield` 指令，这是ARM处理器提供的协作式多任务指令，用于提示当前线程可以暂时让出处理器给其他线程；对于x86/x64架构，函数执行 `rep; nop` 指令序列，这是Intel推荐的暂停指令，能够减少自旋等待时的功耗并改善超线程性能。

在实际应用中，`cpu_relax` 通常被插入到自旋锁的等待循环中，当线程在等待锁释放时，定期调用该函数可以避免CPU资源的浪费，同时给其他线程更多的执行机会。这种机制在Apollo这样的实时系统中特别重要，因为系统需要在保证响应性的同时避免无意义的CPU占用。函数末尾的 `"memory"` 约束告诉编译器该操作可能会影响内存，防止编译器进行可能破坏内存一致性的优化，确保在多线程环境下的正确性。


### 4. CheckedMalloc & CheckedCalloc
```cpp
inline void* CheckedMalloc(size_t size) {
  void* ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

inline void* CheckedCalloc(size_t num, size_t size) {
  void* ptr = std::calloc(num, size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}
```

`CheckedMalloc` 和 `CheckedCalloc` 是对标准C库内存分配函数的安全封装，通过添加异常处理机制来提高内存分配的可靠性和错误处理能力。这两个函数在调用底层的 `malloc` 和 `calloc` 后会检查返回的指针是否为空，如果内存分配失败（通常由于系统内存不足），函数会立即抛出 `std::bad_alloc` 异常而不是返回空指针。这种设计遵循了C++的异常处理机制，使得调用者可以通过异常捕获来统一处理内存分配失败的情况，避免了传统C风格编程中需要在每次内存分配后手动检查空指针的繁琐操作。

