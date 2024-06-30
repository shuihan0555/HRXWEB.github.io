---
title: C++类模板template中的typename使用方法
subtitle:
date: 2024-06-30 13:28:05 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: C/C++ 泛型编程
show_tags: true
---

# 转载

[原文](https://www.cnblogs.com/weiyouqing/p/10538892.html)

# 文内几个重点关注的内容

1. 三个关键概念
    - 限定名与非限定名
    - 依赖名与非依赖名
    - 类作用域
2. 引入typename是为了解决模板中的类型名与成员名冲突问题，即编译时无法确定是类型还是成员名。
