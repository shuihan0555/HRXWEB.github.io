---
title: ROS2注册节点到component时命名空间冲突的问题
subtitle:
date: 2024-01-19 08:12:34 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS
show_tags: true
---

使用 RCLCPP_COMPONENTS_REGISTER_NODE 宏，注册一个节点作为一个可加载组件（loadable component）时，报namespace collision的Warning。
<!--more-->

报错片段：

>[node-3] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rclcpp_components::NodeFactoryTemplate\<XXX\>. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.                                
[node-3]          at line 253 in /XXX/XXX/class_loader_core.hpp

## 原因

PS: 引起这段warning的方式有很多种，但都是因为多次传入宏的参数class_name是一样的所导致的。

下面是我遇到的一种情况：

```cpp
// 冲突的节点
// XXXNode.hpp
class XXXNode : public rclcpp::Node
{
    ...
};
RCLCPP_COMPONENTS_REGISTER_NODE(XXXNode)

// utils.hpp
#include "XXXNode.hpp"

// node1.hpp
#include "utils.hpp"
class Node1 : public rclcpp::Node
{
    ...
};

// node2.hpp
#include "utils.hpp"
class Node2 : public rclcpp::Node
{
    ...
};
```

可以看到，XXXNode.hpp 由于被 utils.hpp 引用(即#include)了，所以在 node1.hpp 和 node2.hpp 中都被引用了，导致了多次调用 RCLCPP_COMPONENTS_REGISTER_NODE(XXXNode)。

## 解决方法

将宏挪到源文件中，而不是头文件中。

```cpp
// XXXNode.hpp
class XXXNode : public rclcpp::Node
{
    ...
};
// RCLCPP_COMPONENTS_REGISTER_NODE(XXXNode)


// XXXNode.cpp
#include "XXXNode.hpp"
...
...
RCLCPP_COMPONENTS_REGISTER_NODE(XXXNode)
```

使用不规范，亲人两行泪。

The End.
