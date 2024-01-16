---
title: 使用Doxygen生成UML
subtitle:
date: 2023-01-03 15:14:31 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: C/C++ STL
show_tags: true
---

<!--more-->

# doxygen生成C++的UML

## 安装工具

```shell
$ sudo apt-get update 
$ sudo apt-get install graphviz doxygen
```

## 使用

```shell
$ cd code_dir
# 生成配置文件
$ doxygen -g Doxygen.config
# 按需修改配置文件
"""
EXTRACT_ALL            = YES
HAVE_DOT               = YES
UML_LOOK               = YES
RECURSIVE              = YES 
"""
# 运行
$ doxygen Doxygen.config
# 生成了两个目录 latex html
# 在 latex 中的 pdf 文件可以找到每个类的继承关系
```

# 参考资料

1. [csdn-Linux下自动生成c++ UML图](https://blog.csdn.net/Cross_Entropy/article/details/117265884)