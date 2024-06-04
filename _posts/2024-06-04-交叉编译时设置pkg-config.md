---
title: 交叉编译时设置pkg-config
subtitle:
date: 2024-06-04 15:14:23 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: CMake cross-compile
show_tags: true
---

交叉编译若使用pkg-config寻找package，需要设置一些环境让pkg-config能够找到交叉编译的库。

<!--more-->

## 问题

若不设置，pkg-config会去默认的路径下寻找库，这样会导致找不到交叉编译的库。会出现类似以下错误：

```shell
CMake Error at /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:463 (message):
  A required package was not found
Call Stack (most recent call first):
  /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:643 (_pkg_check_modules_internal)
  CMakeLists.txt:10 (pkg_check_modules)
```

从错误信息可以看出，CMake在调用`pkg_check_modules`时，pkg-config找不到对应的库。

## 解决方法

参考[这里](https://autotools.info/pkgconfig/cross-compiling.html)。
或者[这里](https://stackoverflow.com/questions/9221236/pkg-config-fails-to-find-package-under-sysroot-directory)

设置几个环境变量

```shell
# sysroot是所有交叉编译文件所在的根目录，具体参考 [cmake sysroot](https://cmake.org/cmake/help/latest/variable/CMAKE_SYSROOT.html)
export SYSROOT=/path/to/sysroot
# 以下的路径自己查看sysroot下到底有没有，填充的思路就是找到pkgconfig目录填进去即可。
export PKG_CONFIG_PATH=
export PKG_CONFIG_LIBDIR=${SYSROOT}/usr/lib/pkgconfig:${SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig
export PKG_CONFIG_SYSROOT_DIR=${SYSROOT}
```

## 为什么要设置这些环境变量

参看pkg-config的 man page 节选：

```shell
ENVIRONMENT VARIABLES
       PKG_CONFIG_PATH
              A colon-separated (on Windows, semicolon-separated) list of directories to search for .pc files.  The default directory will always be searched after searching the path; the default is
              libdir/pkgconfig:datadir/pkgconfig where libdir is the libdir for pkg-config and datadir is the datadir for pkg-config when it was installed.
        PKG_CONFIG_SYSROOT_DIR
              Modify -I and -L to use the directories located in target sysroot.  this option is useful when cross-compiling packages that use pkg-config to determine CFLAGS and LDFLAGS. -I and -L are
              modified to point to the new system root. this means that a -I/usr/include/libfoo will become -I/var/target/usr/include/libfoo with a PKG_CONFIG_SYSROOT_DIR equal to /var/target (same
              rule apply to -L)
        PKG_CONFIG_LIBDIR
              Replaces the default pkg-config search directory, usually /usr/lib/pkgconfig:/usr/share/pkgconfig.
        ...
```

1. PKG_CONFIG_PATH 是pkg-config寻找.pc文件的路径，设置这个变量可以让pkg-config在指定的路径下寻找.pc文件。
2. PKG_CONFIG_SYSROOT_DIR 是用来修改-I和-L的，让pkg-config在交叉编译时能够找到正确的头文件和库文件。
3. PKG_CONFIG_LIBDIR 替代默认的pkg-config搜索路径，这个变量可以让pkg-config在指定的路径下寻找.pc文件。

# 参考

1. [Cross-compiling with pkg-config](https://autotools.info/pkgconfig/cross-compiling.html)
2. [pkg-config fails to find package under sysroot directory](https://stackoverflow.com/questions/9221236/pkg-config-fails-to-find-package-under-sysroot-directory)
