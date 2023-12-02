---
title: CMAKE_SYSROOT和CMAKE_FIND_ROOT_PATH的区别
subtitle:
date: 2023-12-02 22:36:03
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: cmake
show_tags: true

---
<!--more-->
# CMAKE_SYSROOT和CMAKE_FIND_ROOT_PATH的区别

1. CMAKE_FIND_ROOT_PATH用来<font color =red>控制</font>find_\*命令搜索路径的前缀。

   ```cmake
   # adjust the default behavior of the FIND_XXX() commands:
   # search programs in the host environment
   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
   
   # search headers and libraries in the target environment
   set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
   set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
   ```

2. CMAKE_SYSROOT的参数会被find_\*命令<font color =red>用作</font>搜索路径的前缀。并且会传递给编译器的 --sysroot 参数，那么此时究竟作用是什么就取决于编译器了。并且这一参数还有细分：

   1. CMAKE_SYSROOT_COMPILE: used only for compiling sources and not linking.
   2. CMAKE_SYSROOT_LINK: used only for linking and not compiling sources.

# 参考资料

1. [CMAKE_SYSROOT](https://cmake.org/cmake/help/v3.16/variable/CMAKE_SYSROOT.html#cmake-sysroot)
2. [CMAKE_FIND_ROOT_PATH](https://cmake.org/cmake/help/latest/variable/CMAKE_FIND_ROOT_PATH.html)
3. [Cross Compiling With CMake](https://cmake.org/cmake/help/book/mastering-cmake/chapter/Cross%20Compiling%20With%20CMake.html)