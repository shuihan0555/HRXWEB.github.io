---
title: ROS2单个Node的多个callback默认串行执行
subtitle:
date: 2023-12-11 14:02:23 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS
show_tags: true
---
当一个Node存在多个subscriber时，意味着有多个callback函数需要执行，那么他们究竟是串行执行还是并行执行的呢？或者其他的方式？
<!--more-->

# ROS2单个Node的多个callback默认串行执行

## 执行器Executors

用户自定义的Node本身只是有callback成员函数的继承自rclcpp::Node的派生类，并不具有处理消息和时间的能力。

在主线程中，负责处理这些的是执行器--Executor。而在默认情况下，使用的是 `SingleThreadedExecutor`

根据官方资料<sup>1</sup>，如下的代码：

```cpp
int main(int argc, char* argv[])
{
   // Some initialization.
   rclcpp::init(argc, argv);
   ...

   // Instantiate a node.
   rclcpp::Node::SharedPtr node = ...

   // Run the executor.
   rclcpp::spin(node);

   // Shutdown and exit.
   ...
   return 0;
}
```

其中的spin其实是对这一段代码的封装：

```cpp
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

即真正执行轮训或者其他处理调度方式的是executor。默认情况下，具体是 `rclcpp::executors::SingleThreadedExecutor`

其他类型的执行器自行参看官方文档<sup>1</sup>，不在本文讨论范围

## callback_group

在一个node中的所有callback函数，其实都是以组的形式在管理

想要配置callback所属的组别，就要用到对目前的笔者来说，极少使用到的 `rclcpp::SubscriptionOptions`

```cpp
my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions options;
options.callback_group = my_callback_group;

my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(),
                                             callback, options);
```

`create_subscription` 在使用的时候是可以不传入 options 参数的，那此时用的就是默认的参数（意味着默认的callback group）。

而 callback group 分成两种类型：

1. *Mutually exclusive:* Callbacks of this group must not be executed in parallel.
2. *Reentrant:* Callbacks of this group may be executed in parallel.

为了测试默认的究竟是哪种类型，笔者在之前写过的Node的构造函数里面加了一行代码来打印其类型：

```cpp
std::cout << (int)(rclcpp::Node::get_node_base_interface()->get_default_callback_group()->type()) << std::endl;

// 输出的结果是0，而又因为：

enum class CallbackGroupType
{
  MutuallyExclusive,
  Reentrant
};

// 所以默认的是 MutuallyExclusive 类型，即串行方式执行
```

另外具体的调度方式也请参看官方文档<sup>1</sup>

# 参考资料

1. [ROS--About-Executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)

