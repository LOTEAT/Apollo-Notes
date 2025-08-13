<!--
 * @Author: LOTEAT
 * @Date: 2025-08-04 18:40:08
-->

## for_each详解

[知乎链接]()

[Github]()

### 1. for_each

```cpp
DEFINE_TYPE_TRAIT(HasLess, operator<)  // NOLINT

template <class Value, class End>
typename std::enable_if<HasLess<Value>::value && HasLess<End>::value,
                        bool>::type
LessThan(const Value& val, const End& end) {
  return val < end;
}

template <class Value, class End>
typename std::enable_if<!HasLess<Value>::value || !HasLess<End>::value,
                        bool>::type
LessThan(const Value& val, const End& end) {
  return val != end;
}

#define FOR_EACH(i, begin, end)           \
  for (auto i = (true ? (begin) : (end)); \
       apollo::cyber::base::LessThan(i, (end)); ++i)
```

这里是对`DEFINE_TYPE_TRAIT`的一个应用，首先通过

```cpp
DEFINE_TYPE_TRAIT(HasLess, operator<)
```

定义了一个HasLess结构体，这个结构的作用是判断某个类型中是否存在小于这个运算符。

`FOR_EACH` 宏是一个循环构造工具，为不同类型的迭代器提供统一的循环接口。该宏通过 `HasLess` 类型特征检测迭代器类型是否支持小于运算符（`operator<`），然后利用 SFINAE 技术选择合适的 `LessThan` 函数重载：对于支持小于运算符的类型，使用 `val < end` 进行比较；对于不支持小于运算符的类型（如输入迭代器、前向迭代器），则使用 `val != end` 进行比较，确保兼容性。宏中的 `(true ? (begin) : (end))` 是一个类型推导技巧，它确保循环变量 `i` 的类型与 `begin`和`end` 一致。

例如：

```cpp
int* a;
const int* b;
auto x = a;              // x 是 int*
auto y = b;              // y 是 const int*
auto z = true ? a : b;   // z 是 const int*
```

这样可以强制编译器推导一个 `begin` 和 `end` 共同的类型，即使它们本身是不同的类型。



Reference:

