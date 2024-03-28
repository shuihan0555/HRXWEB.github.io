---
title: std::unique_ptr使用incomplete type的报错分析和解决
subtitle:
date: 2024-03-28 15:16:58 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: C/C++
show_tags: true
---

对于不完整类型使用`std::unique_ptr`时，会出现编译错误，本文分析了这个问题的原因，并提供了解决方案。
<!--more-->

# 本文前半部分转载自[此博客](http://www.misteo.top/2020/11/30/std-unique-ptr使用incomplete-type的报错分析和解决/)

Pimpl(Pointer to implementation)很多同学都不陌生，但是从原始指针升级到C++11的独占指针std::unique_ptr时，会遇到一个incomplete type的报错，本文来分析一下报错的原因以及分享几种解决方法~

## 问题现象

首先举一个传统C++中的Pimpl的例子

```cpp
// widget.h

// 预先声明
class Impl;

class Widget
{
    Impl * pImpl;
};
```

很简单，没什么问题，但是使用的是原始指针，现在我们升级到std::unique_ptr

```cpp
// widget.h

// 预先声明
class Impl;

class Widget
{
    std::unique_ptr<Impl> pImpl;
};
```

很简单的一次升级，而且也能通过编译，看似也没问题，但当你创建一个Widget的实例

```cpp
// pimpl.cpp

#include "widget.h"

Widget w;
```

这时候，问题来了

```shell
$ g++ pimpl.cpp
In file included from /usr/include/c++/9/memory:80,
    from widget.h:1,
    from pimpl.cpp:1:
/usr/include/c++/9/bits/unique_ptr.h:
    In instantiation of ‘void std::default_delete<_Tp>::operator()(_Tp*) const [with _Tp = Impl]’:
/usr/include/c++/9/bits/unique_ptr.h:292:17:
    required from ‘std::unique_ptr<_Tp, _Dp>::~unique_ptr() [with _Tp = Impl; _Dp = std::default_delete<Impl>]’
widget.h:5:7:   required from here
/usr/include/c++/9/bits/unique_ptr.h:79:16: error: invalid application of ‘sizeof’ to incomplete type ‘Impl’
79 |  static_assert(sizeof(_Tp)>0,
   |                ^~~~~~~~~~~
```

## 原因分析

从报错我们可以看出，std::unique_ptr中需要静态检测类型的大小static_assert(sizeof(Impl)>0，但是我们的Impl是一个预先声明的类型，是incomplete type，也就没法计算，所以导致报错。

想要知道怎么解决，首先需要知道std::unique_ptr为啥需要计算这个，我们来看一下STL中相关的源码，从报错中得知是unique_ptr.h的292行，调用了79行，我们把前后相关源码都粘出来（来自g++ 9.3.0中的实现）

```cpp
// 292行附近

      /// Destructor, invokes the deleter if the stored pointer is not null.
      ~unique_ptr() noexcept
      {
    static_assert(__is_invocable<deleter_type&, pointer>::value,
              "unique_ptr's deleter must be invocable with a pointer");
    auto& __ptr = _M_t._M_ptr();
    if (__ptr != nullptr)
// 292行在这里
      get_deleter()(std::move(__ptr));
    __ptr = pointer();
      }

// 79行附近

  /// Primary template of default_delete, used by unique_ptr
  template<typename _Tp>
    struct default_delete
    {
      /// Default constructor
      constexpr default_delete() noexcept = default;

      /** @brief Converting constructor.
       *
       * Allows conversion from a deleter for arrays of another type, @p _Up,
       * only if @p _Up* is convertible to @p _Tp*.
       */
      template<typename _Up, typename = typename
           enable_if<is_convertible<_Up*, _Tp*>::value>::type>
        default_delete(const default_delete<_Up>&) noexcept { }

      /// Calls @c delete @p __ptr
      void
      operator()(_Tp* __ptr) const
      {
    static_assert(!is_void<_Tp>::value,
              "can't delete pointer to incomplete type");
// 79行在这里
    static_assert(sizeof(_Tp)>0,
              "can't delete pointer to incomplete type");
    delete __ptr;
      }
    };
```

std::unique_ptr中的析构函数，调用了默认的删除器default_delete，而default_delete中检查了Impl，其实就算default_delete中不检查，到下一步delete __ptr;，还是会出问题，因为不完整的类型无法被delete。

## 解决方法

原因已经知道了，那么解决方法就呼之欲出了，这里提供三种解决方法：

1. 方法一：改用std::shared_ptr
2. 方法二：自定义删除器，将delete pImpl的操作，放到widget.cpp源文件中
3. 方法三：仅声明Widget的析构函数，但不要在widget.h头文件中实现它

其中我最推荐方法三，它不改变代码需求，且仅做一点最小的改动，下面依次分析

### 方法一：改用std::shared_ptr

```cpp
// widget.h

// 预先声明
class Impl;

class Widget
{
    std::shared_ptr<Impl> pImpl;
};
```

改完就能通过编译了，这种改法最简单。但是缺点也很明显：使用shared_ptr可能会改变项目的需求，shared_ptr也会带来额外的性能开销，而且违反了“尽可能使用unique_ptr而不是shared_ptr”的原则（当然这个原则是我编的，哈哈）

那为什么unique_ptr不能使用预先声明的imcomplete type，但是shared_ptr却可以？

因为对于unique_ptr而言，删除器是类型的一部分：

```cpp  
template<typename _Tp, typename _Dp>
    class unique_ptr<_Tp[], _Dp>
```

这里的_Tp是element_type，_Dp是deleter_type

而shared_ptr却不是这样：

```cpp
template<typename _Tp>
    class shared_ptr : public __shared_ptr<_Tp>
```

那为什么unique_ptr的删除器是类型的一部分，而shared_ptr不是呢？

答案是设计如此！哈哈，说了句废话。具体来说，删除器不是类型的一部分，使得你可以对同一种类型的shared_ptr，使用不同的自定义删除器

```cpp
auto my_deleter = [](Impl * p) {...};

std::shared_ptr<Impl> w1(new Impl, my_deleter);
std::shared_ptr<Impl> w2(new Impl); // default_deleter

w1 = w2; // It's OK!
```

看到了么，这里的两个智能指针w1和w2，虽然使用了不同的删除器，但他们是同一种类型，可以相互进行赋值等等操作。而unique_ptr却不能这么玩

```cpp
auto my_deleter = [](Impl * p) {...};

std::unique_ptr<Impl, decltype(my_deleter)> w1(new Impl, my_deleter);
std::unique_ptr<Impl> w2(new Impl); // default_deleter

// w1的类型是 std::unique_ptr<Impl, lambda []void (Impl *p)->void>
// w2的类型是 std::unique_ptr<Impl, std::default_delete<Impl>>

w1 = std::move(w2); // 错误！类型不同，没有重载operator=
```

道理我都明白了，那为什么要让这两种智能指针有这样的区别啊？

答案还是设计如此！哈哈，具体来说unique_ptr本身就只是对原始指针的简单封装，这样做不会带来额外的性能开销。而shared_ptr的实现提高了灵活性，但却进一步增大了性能开销。针对不同的使用场景所以有这样的区别。

### 方法二：自定义删除器，将delete pImpl的操作，放到widget.cpp源文件中

```cpp
// widget.h

// 预先声明
class Impl;

class Widget
{
    struct ImplDeleter final
    {
        constexpr ImplDeleter() noexcept = default;
        void operator()(Impl *p) const;
    };
    std::unique_ptr<Impl, ImplDeleter> pImpl = nullptr;
};
```

然后在源文件widget.cpp中

```cpp
#include "widget.h"
#include "impl.h"

void Widget::ImplDeleter::operator()(Impl *p) const
{
    delete p;
}
```

这种方法改起来也不复杂，但是弊端也很明显，std::make_unique没法使用了，只能自己手动new，直接看源码吧

```cpp
template<typename _Tp, typename... _Args>
    inline typename _MakeUniq<_Tp>::__single_object
    make_unique(_Args&&... __args)
    { return unique_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...)); }
```

看出问题在哪了么？这里返回的是默认删除器类型的unique_ptr，即std::unique_ptr<Impl, std::default_delete<Impl>>，如方法一中所说，是不同删除器类型的unique_ptr是没法相互赋值的，也就是说：

```cpp
pImpl = std::make_unique<Impl>(); // 错误！类型不同，没有重载operator=

pImpl = std::unique_ptr<Impl, ImplDeleter>(new Impl); // 正确！每次你都要写这么一大串
```

当然你也可以实现一个make_impl，并且using一下这个很长的类型，比如：

```cpp
using unique_impl = std::unique_ptr<Impl, ImplDeleter>;

template<typename... Ts>
unique_impl make_impl(Ts && ...args)
{
    return unique_impl(new Impl(std::forward<Ts>(args)...));
}

// 调用
pImpl = make_impl();
```

看似还凑合，但总的来说，这样做还是感觉很麻烦。并且有一个很头疼的问题：make_impl作为函数模板，没法声明和定义分离，而且其中的用到了new，需要完整的Impl类型。所以，你只能把这一段模板函数写在源文件中，emmm，总感觉不太对劲。

### 方法三：仅声明Widget的析构函数，但不要在widget.h头文件中实现它

```cpp
// widget.h

// 预先声明
class Impl;

class Widget
{
    Widget();
    ~Widget();  // 仅声明

    std::unique_ptr<Impl> pImpl;
};


// widget.cpp
#include "widget.h"
#include "impl.h"

Widget::Widget()
    : pImpl(nullptr)
{}

Widget::~Widget() = default;    // 在这里定义
```

这样就解决了！是不是出乎意料的简单！并且你也可以正常的使用std::make_unique来进行赋值。唯一的缺点就是你没法在头文件中初始化pImpl了

但也有别的问题，因为不光是析构函数中需要析构std::unique_ptr，还有别的也需要，比如移动构造、移动运算符等。所以在移动构造、移动运算符中，你也会遇到同样的编译错误。解决方法也很简单，同上面一样：

```cpp
// widget.h

// 预先声明
class Impl;

class Widget
{
    Widget();
    ~Widget();

    Widget(Widget && rhs);  // 同析构函数，仅声明
    Widget& operator=(Widget&& rhs);

    std::unique_ptr<Impl> pImpl;
};


// widget.cpp
#include "widget.h"
#include "impl.h"

Widget::Widget()
    : pImpl(nullptr)
{}

Widget::~Widget() = default;

Widget(Widget&& rhs) = default;             //在这里定义
Widget& operator=(Widget&& rhs) = default;
```

搞定！

----

----

# 本文后半部分为原创内容

## 需求背景

在一个项目中，有三个头文件的依赖关系一开始是这样的：`A.hpp` -> `B.hpp` -> `C.hpp`，x -> y 表示 x 依赖于 y。

B中会涉及到本文探讨的编译错误的内容可以抽象为如下：

```cpp
// B.hpp
#include <memory>
#include "C.hpp"

Class B
{
    struct Param
    {
        // ...
    };
    B(std::unique_ptr<Param> &&param) {
        this->param = std::move(param);
    }
    ~B() = default;
    a_func(std::unique_ptr<C> &&c) {
        this->c = std::move(c);
        // do something
    }
    // ...
    private:
        std::unique_ptr<Param> param;
        std::unique_ptr<C> c;
};
```

也就是在B头文件中，只用到了 `Class C` 的声明，没有用到 `Class C` 的任何函数。

如果此时要单独编译A的话，不仅需要包含B的头文件，还需要包含C的头文件，这样就会导致C中的内容暴露给A，这是不合理的。

因此想到了一个解决办法就是在B的头文件中使用 incomplete type，即只前向声明 `Class C`，不包含 `C.hpp`，这样就可以避免暴露C的内容给A。包含C的头文件的操作放在B的源文件中。

```cpp
// B.hpp
#include <memory>

class C; // 前向声明

Class B
{
    struct Param
    {
        // ...
    };
    B(std::unique_ptr<Param> &&param) {
        this->param = std::move(param);
    }
    ~B() = default;
    a_func(std::unique_ptr<C> &&c) {
        this->c = std::move(c);
        // do something
    }
    // ...
    private:
        std::unique_ptr<Param> param;
        std::unique_ptr<C> c;
};
```

这时编译器就会报错，报的错误即本文前半部分所述的错误。

经过前文的阐述，可以在这里分析为什么会报 `invalid application of ‘sizeof’ to incomplete type ‘Impl’` 的错误。

1. B中构造函数，虽然没有和 c 相关的代码，（不知道为什么保留在这还是会报这个错，存疑）
2. B的析构函数，因为 `std::unique_ptr` 的析构函数需要调用 `std::default_delete` 的 `operator()`，而 `std::default_delete` 中有 `static_assert(sizeof(_Tp)>0, "can't delete pointer to incomplete type");`，所以会报错。
3. B的 `a_func` 函数，因为 `std::unique_ptr` 的移动赋值操作需要调用 `std::default_delete` 的 `operator()`，而 `std::default_delete` 中有 `static_assert(sizeof(_Tp)>0, "can't delete pointer to incomplete type");`，所以会报错。

采用前文所述的方法三来解决：

```cpp
// B.hpp
#include <memory>

class C; // 前向声明

Class B
{
    struct Param
    {
        // ...
    };
    B(std::unique_ptr<Param> &&param);
    ~B();
    a_func(std::unique_ptr<C> &&c);
    // ...
    private:
        std::unique_ptr<Param> param;
        std::unique_ptr<C> c;
};
```

```cpp
// B.cpp
#include "B.hpp"
#include "C.hpp"

B::B(std::unique_ptr<Param> &&param) {
    this->param = std::move(param);
}

B::~B() = default;

B::a_func(std::unique_ptr<C> &&c) {
    this->c = std::move(c);
    // do something
}
```

这样就解决了问题。