---
title: ld编译选项-rpath和-rpath-link的区别
subtitle:
date: 2023-12-10 11:00:29 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: gcc/g++
show_tags: true
---

<!--more-->

# ld编译选项-rpath和-rpath-link的区别

如资料<sup>1</sup>所述：

> The difference between `-rpath` and `-rpath-link` is that directories specified by `-rpath` options are included in the executable and used at runtime, whereas the `-rpath-link` option is only effective at link time.

1. -rpath-link 只影响静态编译的链接阶段
2. -rpath 还会影响动态运行阶段。它会把路径写到ELF文件的DT_RPATH标记里面，从而影响动态链接器ld.so寻找共享库的过程，但是ld.so的manual不建议这么做。

# 参考资料

1. [using ld](https://ftp.gnu.org/old-gnu/Manuals/ld-2.9.1/html_mono/ld.html#TOC3)