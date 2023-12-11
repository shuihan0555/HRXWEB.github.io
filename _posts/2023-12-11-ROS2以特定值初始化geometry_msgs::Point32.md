---
title: ROS2以特定值初始化geometry_msgs::Point32
subtitle:
date: 2023-12-11 13:29:11 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS
show_tags: true
---

<!--more-->

# ROS2以特定值初始化geometry_msgs::Point32

对于ros2的geometry_msgs::msg::Point32来说，其默认的构造函数都是使用initializer将成员 x y z 都初始化为 0，那么有没有办法直接初始化为特定的值呢？资料<sup>1</sup>提供了一种方法：

```cpp
// 在 point32_builder.hpp 中提供了如何直接以特定的值初始化point32
struct P
{
  float x;
  float y;
  float z;
}p;
geometry_msgs::build<geometry_msgs::msg::Point32>().x(p.x).y(p.y).z(p.z);
```

下面给出point32__builder.hpp的文件内容，一看就明白了：
```cpp
// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from geometry_msgs:msg/Point32.idl
// generated code does not contain a copyright notice

#ifndef GEOMETRY_MSGS__MSG__DETAIL__POINT32__BUILDER_HPP_
#define GEOMETRY_MSGS__MSG__DETAIL__POINT32__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace geometry_msgs
{

namespace msg
{

namespace builder
{

class Init_Point32_z
{
public:
  explicit Init_Point32_z(::geometry_msgs::msg::Point32 & msg)
  : msg_(msg)
  {}
  ::geometry_msgs::msg::Point32 z(::geometry_msgs::msg::Point32::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::geometry_msgs::msg::Point32 msg_;
};

class Init_Point32_y
{
public:
  explicit Init_Point32_y(::geometry_msgs::msg::Point32 & msg)
  : msg_(msg)
  {}
  Init_Point32_z y(::geometry_msgs::msg::Point32::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Point32_z(msg_);
  }

private:
  ::geometry_msgs::msg::Point32 msg_;
};

class Init_Point32_x
{
public:
  Init_Point32_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point32_y x(::geometry_msgs::msg::Point32::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point32_y(msg_);
  }

private:
  ::geometry_msgs::msg::Point32 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::geometry_msgs::msg::Point32>()
{
  return geometry_msgs::msg::builder::Init_Point32_x();
}

}  // namespace geometry_msgs

#endif  // GEOMETRY_MSGS__MSG__DETAIL__POINT32__BUILDER_HPP_

```



# 参考资料

1. [Is there a concise way to initialize a message with specific values?](https://answers.ros.org/question/399059/is-there-a-concise-way-to-initialize-a-message-with-specific-values/)