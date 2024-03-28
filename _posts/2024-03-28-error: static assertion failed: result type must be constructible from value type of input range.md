---
title: 类中std::vector<std::unique_ptr<T>>成员变量带来的问题
subtitle:
date: 2024-03-28 12:23:34 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: C/C++
show_tags: true
---

error: static assertion failed: result type must be constructible from value type of input range
  127 |       static_assert(is_constructible<_ValueType2, decltype(*__first)>::value,
<!--more-->

# 问题复现

1. cmake 3.16
2. gcc 9.4.0
3. ubuntu 20.04

## main.cpp:

```cpp
#include <iostream>
#include <vector>
#include <memory>

struct Test {
    float x;
    Test() = default;
    Test(const Test&) = default;
    Test& operator=(const Test&) = default;
    Test(Test&&) = default;
    Test& operator=(Test&&) = default;
};

class MyClass {
public:
    MyClass(int count) : count(count) {}
    // MyClass(const MyClass&) = delete;
    ~MyClass() = default;
    
private:
    int count;  // 基本类型成员变量
    std::vector<std::unique_ptr<Test>> vec;
};

int main()
{
    auto obj = MyClass(10);
    return 0;
}
```

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.12)
project(test_unique)

set(CMAKE_CXX_STANDARD 14)

add_executable(main main.cpp)
```

## 编译错误

```shell
mkdir build
cd build
cmake ..
make
```

```plaintext
In file included from /usr/include/c++/9/vector:66,
                 from /home/huangruixin/Downloads/test_unique/main.cpp:2:
/usr/include/c++/9/bits/stl_uninitialized.h: In instantiation of ‘_ForwardIterator std::uninitialized_copy(_InputIterator, _InputIterator, _ForwardIterator) [with _InputIterator = __gnu_cxx::__normal_iterator<const std::unique_ptr<Test>*, std::vector<std::unique_ptr<Test> > >; _ForwardIterator = std::unique_ptr<Test>*]’:
/usr/include/c++/9/bits/stl_uninitialized.h:307:37:   required from ‘_ForwardIterator std::__uninitialized_copy_a(_InputIterator, _InputIterator, _ForwardIterator, std::allocator<_Tp>&) [with _InputIterator = __gnu_cxx::__normal_iterator<const std::unique_ptr<Test>*, std::vector<std::unique_ptr<Test> > >; _ForwardIterator = std::unique_ptr<Test>*; _Tp = std::unique_ptr<Test>]’
/usr/include/c++/9/bits/stl_vector.h:555:31:   required from ‘std::vector<_Tp, _Alloc>::vector(const std::vector<_Tp, _Alloc>&) [with _Tp = std::unique_ptr<Test>; _Alloc = std::allocator<std::unique_ptr<Test> >]’
/home/huangruixin/Downloads/test_unique/main.cpp:14:7:   required from here
/usr/include/c++/9/bits/stl_uninitialized.h:127:72: error: static assertion failed: result type must be constructible from value type of input range
  127 |       static_assert(is_constructible<_ValueType2, decltype(*__first)>::value,
      |                                                                        ^~~~~
make[2]: *** [CMakeFiles/main.dir/build.make:63: CMakeFiles/main.dir/main.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:76: CMakeFiles/main.dir/all] Error 2
make: *** [Makefile:84: all] Error 2
```

# 问题分析

铁出来的 error log 前两行是这样的：

```plaintext    
In file included from /usr/include/c++/9/vector:66,
                 from /home/huangruixin/Downloads/test_unique/main.cpp:2:
```

从这里面根本看不出来任何的信息，`main.cpp` 的第二行是 `#include <vector>`，这个信息对于我们来说没有任何帮助。

从后面的 log 还是能看出来和 `std::vector<std::unique_ptr<Test>>` 有关，但是具体是什么问题呢？

再分析后面的 log，发现调用的链路是这样的：

```plaintext
main.cpp:14:7 -> stl_vector.h:555:31 -> stl_uninitialized.h:307:37 -> stl_uninitialized.h:127:72
```

重点就是 `main.cpp:14:7` 根本没有提供任何有用的信息，这个位置就是类的声明而已。

最终偶然发现是调用了编译器生成的 `MyClass` 的拷贝构造函数，里面操作了 `std::vector<std::unique_ptr<Test>>`，这个就是问题的根源。

## 解决方案

1. 删除拷贝构造函数和拷贝赋值函数
2. 自定义拷贝构造函数和拷贝赋值函数
