---
title: CMake的public/private/interface和可视性/传递性的关系
subtitle:
date: 2024-04-30 06:30:00 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: CMake
show_tags: true
---


<!--more-->

## 详细描述

[此博客](https://chunleili.github.io/cmake/understanding-INTERFACE)

以下摘抄自上述博客：

## 总结

以 A 依赖 B 为例。

1. PUBLIC的意思就是 目标B的属性 不仅自己使用，还传递给依赖它的目标A。
2. PRIVATE的意思就是 目标B的属性 不会传递，只给目标B自己使用。
3. 而INTERFACE则极为特殊：它的属性都 不会自己使用，只传递给目标A。

## INTERFACE的实现机制

我们还可以从它内部的实现机制来理解INTERFACE是什么

我们这里就以target_include_directories为例，其他的属性同理。

在cmake内部，有两个变量：INCLUDE_DIRECTORIES 和 INTERFACE_INCLUDE_DIRECTORIES 。

这两个变量存储的都是头文件目录。和系统的PATH变量类似，里面可以有多个路径，以分号分割。

但是区别是：INCLUDE_DIRECTORIES 是**当前目标搜索的头文件目录**，而INTERFACE_INCLUDE_DIRECTORIES 是**下一个目标要搜索的头文件目录**。

当目标B去搜索头文件的时候，就会在INCLUDE_DIRECTORIES 中搜索。这是简单清晰的。

而假如A引用了B（或者说目标A依赖于目标B），那么INTERFACE_INCLUDE_DIRECTORIES 中的路径就会赋给目标A的INCLUDE_DIRECTORIES。

所以，使用PRIVATE PUBLIC和INTERFACE就能控制是否将当前搜索路径传递给下一个目标。

PRIVATE就是不把当前的INCLUDE_DIRECTORIES 传递给INTERFACE_INCLUDE_DIRECTORIES 。

PUBLIC就是把当前的INCLUDE_DIRECTORIES 传递给INTERFACE_INCLUDE_DIRECTORIES 。

INTERFACE就是自己不使用当前的INCLUDE_DIRECTORIES ，但是把当前的INCLUDE_DIRECTORIES 传递给 INTERFACE_INCLUDE_DIRECTORIES 。
