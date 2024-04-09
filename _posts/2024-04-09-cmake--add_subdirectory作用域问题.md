---
title: CMake--add_subdirectory使用域问题
subtitle:
date: 2024-04-09 12:15:23 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: CMake
show_tags: true
---

<!--more-->

> 和 include() 的区别
> 
> include 类似于 cpp 的 #include，将被包含的文件的内容直接插入到包含文件中，
> 
> 而 add_subdirectory() 则是将被包含的文件作为一个独立的项目进行构建，主目录的变量是拷贝一份给子目录的，子目录的变量不会影响主目录的变量。

# CMake--add_subdirectory使用域问题

这里要说的的作用域是指在subdirectory中生成的target，能否在主目录及其他subdirectory中使用。

<font color = red>**结论：**</font>在subdirectory中生成的target，能在主目录及其他subdirectory中使用。

gpt的解释：

当执行add_subdirectory(A)命令时，CMake会处理子目录A的CMakeLists.txt文件，并将子目录A的构建过程包含在主目录的构建过程中。这意味着子目录A中定义的库A将会在主目录的构建过程中生成，并可以在主目录及其子目录中使用。

## 验证

```plaintext
- CMakeLists.txt
- A/
  - CMakeLists.txt
  - a.cpp
  - a.h
- B/
  - CMakeLists.txt
  - main.cpp
```

### CMakelists.txt

```cmake
cmake_minimum_required(VERSION 3.0)
project(MyProject)

add_subdirectory(A)
add_subdirectory(B)

### A/CMakeLists.txt

```cmake
add_library(A a.cpp a.h)
```

### A/a.h
    
```cpp
#ifndef A_H
#define A_H

class A {
public:
    void foo();
};

#endif // A_H
```

### A/a.cpp

```cpp
#include "a.h"
#include <iostream>

void A::foo() {
    std::cout << "Hello from A!" << std::endl;
}
```

### B/CMakeLists.txt

```cmake
add_executable(B main.cpp)
# link A to B
target_link_libraries(B A)
```

### B/main.cpp

```cpp
#include <iostream>
#include "a.h"

int main() {
    A a;
    a.foo();
    return 0;
}
```

## 编译

```shell
$ mkdir build
$ cd build
$ cmake ..
$ make
```

最终可以编译通过，说明在B目录中可以使用A目录生成的target。

## 运行

```shell
$ ./B/B
```

输出：

```shell
Hello from A!
```

## 补充

如果交换 `add_subdirectory(A)` 和 `add_subdirectory(B)` 的顺序，编译依然可以通过，说明在构建过程不是严格按照顺序进行的，会对依赖关系进行处理。

# 参考资料

1. [CMake(六)：使用子目录](https://blog.csdn.net/jjjstephen/article/details/122455635)