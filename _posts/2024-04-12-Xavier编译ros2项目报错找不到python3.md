---
title: Xavier编译ros2项目报错找不到python3
subtitle:
date: 2024-04-12 07:53:21 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: Jetson Xavier ROS
show_tags: true
---

<!--more-->

## 开发环境

1. Ubuntu 20.04
2. AGX Xavier
3. Jetpack 5.1.3
4. 我司魔改的ros2humble

## 具体报错

```plaintext
colcon build --packages-up-to cv_bridge nov_codec nvm_msgs idps_msgs nova_msgs idps_perception_node
Starting >>> nvm_msgs
Starting >>> cv_bridge
Starting >>> idps_msgs                                                
Starting >>> nova_msgs
--- stderr: cv_bridge                                                                                                               
CMake Error at /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
  Could NOT find Python3 (missing: Python3_LIBRARIES Python3_INCLUDE_DIRS
  Python3_NumPy_INCLUDE_DIRS Development NumPy) (found version "3.9.5")
Call Stack (most recent call first):
  /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:393 (_FPHSA_FAILURE_MESSAGE)
  /usr/share/cmake-3.16/Modules/FindPython/Support.cmake:2214 (find_package_handle_standard_args)
  /usr/share/cmake-3.16/Modules/FindPython3.cmake:300 (include)
  CMakeLists.txt:19 (find_package)
...
...
```

## 错误分析

是有找到python3.9.5(found version "3.9.5")的，但是相关的库文件，头文件都找不到。但是使用 `which python3` 发现 python指向的 python3.8。

进一步发现 `/usr/lib/python3.9` 下没有 `dist-packages` 目录。python3.9 的 pip 安装的 numpy 位于 `/usr/lib/python3/dist-packages` 这个路经。

比较奇怪。

## 解决方案

所以选择将 python3.9 先屏蔽掉：

```shell
$ sudo mv /usr/bin/python3.9 /usr/bin/python3.9.bak
```

顺利解决～
