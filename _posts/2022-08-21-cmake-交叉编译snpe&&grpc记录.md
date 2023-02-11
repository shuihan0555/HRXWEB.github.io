---
title: 交叉编译snpe&&grpc记录
subtitle: 
date: 2022-08-21 19:07:48
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: gRPC snpe cmake
show_tags: true

---

笔者从这个工作开始接触 cmake ，期间踩了不少坑，现在也算是成功达到了半吊子的水平。

整理了几篇笔记之后，觉得有些零散。因此借由这篇详尽的例子，做到理论和实际相结合。

# 交叉编译 SNPE

由于本组项目需要，受限于硬件无法接显示屏，数据可视化需要借助 protobuf 做成标准格式，而后通过 grpc 远程调用显示。在本机进行开发时，就需要交叉编译生成可执行文件到 arm 板子上运行。

# 交叉编译 grpc

<font color = purple>依据2022.08.25--[grpc官方库](https://github.com/grpc/grpc/blob/master/test/distrib/cpp/run_distrib_test_cmake_aarch64_cross.sh)脚本59行～62行可知，在交叉编译 grpc 之前，需要依赖本机的 protoc 和 grpc_cpp_plugin 等可执行文件，因此需要先在本机编译安装 grpc （这个过程也会安装好protoc，不需要单独去安装）。</font>

## 安装适用于本地x86的grpc

[参考](https://grpc.io/docs/languages/cpp/quickstart/)

```bash
# **** 准备工作 **** #
# 定义一个环境变量，关掉此终端就会失效了，如果想长期有效，可以把这句话复制到 ~/.bashrc 并 $ source ~/.bahsrc
$ export MY_INSTALL_DIR=<PATH_DEFINED_BY_USER>
# 建立文件夹， -p 参数，递归的创建。
$ mkdir -p $MY_INSTALL_DIR
# 安装好之后，会有可执行文件放在 $MY_INSTALL_DIR/bin 目录下，加到 PATH 里面，可执行文件才能被找到并运行。同样想长期有效，那就复制到 bashrc 中
$ export PATH="$MY_INSTALL_DIR/bin:$PATH"
# **** 准备工作 **** #

# **** 安装 cmake **** #
# 下载安装脚本
$ wget -q -O cmake-linux.sh https://github.com/Kitware/CMake/releases/download/v3.19.6/cmake-3.19.6-Linux-x86_64.sh
# 运行脚本，指定安装位置：--prefix=/usr/local
$ sh cmake-linux.sh --skip-license --prefix=/usr/local
# optional，上面指定的位置一般情况下都在 PATH 这个环境变量里面了，如果读者 --prefix 中定义的路径里面没有在 PATH 中，由于经常需要用到 cmake，可以将下面一条命令执行一次，注意 '>>' 是追加， '>' 是覆盖。一定要用追加的方式，否则 bashrc以前的内容都被删了。
$ echo $PATH="<path>/bin:$PATH" >> ~/.bashrc && source ~/.bashrc # [optional]
# 删除脚本
$ rm cmake-linux.sh
# **** 安装 cmake **** #

# **** 编译本地版本的 grpc **** #
# 安装依赖
$ sudo apt install -y build-essential autoconf libtool pkg-config
# clone grpc 仓库，并且选择 v1.46.3 这个分支
$ git clone --recurse-submodules -b v1.46.3 --depth 1 --shallow-submodules https://github.com/grpc/grpc
# change directory
$ cd grpc
# 递归创建文件夹
$ mkdir -p cmake/build
# 将这个目录加入到目录栈中
$ pushd cmake/build
# 预编译，注意 -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR ！！！
$ cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
      ../..
# 编译 并 安装，其中安装会将生成的 lib、bin、include 等目录里面的内容移动到安装目录下的 bin、lib、inlcude目录中，不会覆盖！！！！
$ make -j2 install
# 释放目录栈
$ popd
# **** 编译本地版本的 grpc **** #
```

## 编译 aarch64 版本 grpc

```bash
# 安装编译工具链
$ sudo apt-get install g++-aarch64-linux-gnu
# 设定工具链的配置文件，写到 /tmp/toolchain 中
$ cat > /tmp/toolchain.cmake <<'EOT'
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_STAGING_PREFIX /tmp/stage)
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
EOT
# 此时应该位于clone下来的grpc这个目录！！！！
$ mkdir -p cmake/build_arm
$ pushd cmake/build_arm
# 加参数，1. 工具链设定文件 2. build 类型 3. 安装的位置，注意最后会安装到 set(CMAKE_STAGING_PREFIX /tmp/stage) 这个设定下的 /tmp/stage 目录，目前并不清楚为什么要这么做。
$ cmake -DCMAKE_TOOLCHAIN_FILE=/tmp/toolchain.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/tmp/install \
      ../..
# 编译安装
$ make -j2 install
$ popd
```

# 交叉编译 SNPE

##  设定工具链

```bash
# 将此文件放在和 CMakeLists.txt 同级的目录。当然可以随便放位置，到时候可以通过绝对路径索引。
$ cat > aarch64_linux_setup.cmake << 'EOT'
SET(CMAKE_SYSTEM_NAME Linux)		# 板子操作系统是Linux
SET(CMAKE_SYSTEM_PROCESSOR aarch64)		# 板子架构是aarch64
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)		# C 编译器
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)		# CPP 编译器
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)		# program不在指定的路径下寻找
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)		# lib 只在指定路径寻找
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)		# include 只在指定路径寻找
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)		# package 只在指定路径寻找
EOT
```

## ‼️<font color =red>编写 CMakeLists.txt</font>

ps：由于这个项目的目录层级嵌套关系十分深，因此需要使用一些略微高级的功能。避免每次都要 cd 到最里层的目录进行 build。

### 项目结构

```bash
$ sudo apt install tree
$ cd <ROOT_PATH_OF_PROJECT>
$ tree -a
>>>
.
├── CMakeLists.txt		!‼️!
├── NativeCpp
│   ├── mvlidarnet
│   │   ├── 3dnms.cpp
│   │   ├── 3dnms.hpp
│   │   ├── aarch64_oe_linux_setup.cmake
│   │   ├── build		!‼️!
│   │   ├── CheckRuntime.cpp
│   │   ├── CheckRuntime.hpp
│   │   ├── CMakeLists.txt		!‼️!
│   │   ├── config
│   │   │   ├── pqr_kf_cv_exp.yaml
│   │   │   ├── pqr_kf_cv.yaml
│   │   │   └── tracking_config_nova3d.yaml
│   │   ├── CreateUserBuffer.cpp
│   │   ├── CreateUserBuffer.hpp
│   │   ├── LoadContainer.cpp
│   │   ├── LoadContainer.hpp
│   │   ├── LoadInputTensor.cpp
│   │   ├── LoadInputTensor.hpp
│   │   ├── main.cpp
│   │   ├── PreprocessInput.cpp
│   │   ├── PreprocessInput.hpp
│   │   ├── SaveOutputTensor.cpp
│   │   ├── SaveOutputTensor.hpp
│   │   ├── SetBuilderOptions.cpp
│   │   ├── SetBuilderOptions.hpp
│   │   ├── Util.cpp
│   │   └── Util.hpp
├── README.md

XXX directories, XXX files

# 注意三个感叹号的位置
# 1. 最外层的 <PATH_OF_PROJECT>/CMakeLists.txt
# 2. 里层的 <PATH_OF_PROJECT>/NativeCpp/mvlidarnet/CMakeLists.txt
# 3. build目录，一般都是在这里面存放编译过程产生的文件 <PATH_OF_PROJECT>/NativeCpp/mvlidarnet/build/
```

### 外层 CMakeLists.txt

```cmake
# cmake 最低版本
cmake_minimum_required(VERSION 3.16)
# 项目名称，此时自动生成两个变量 PROJECT_NAME 和 PROJECR_SOURCE_DIR 代表本目录。
project(road865bx)
# c++ 标准
set(CMAKE_CXX_STANDARD 14)
# 一些 cmake 默认存在的变量的设置
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-march=native -O2")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
# 给 cross_compile 加一个开关
option(CROSS_COMPILE "This is an option for cross compile" OFF)

# !‼️! 这就是多层级目录下有多个 CMakeLists.txt 时的用法
add_subdirectory(NativeCpp/mvlidarnet)
# 另外补充，若子目录下生成了 动/静态 库，可以在这个文件中链接这些库生成可执行文件。可以看参考资料3
```

### 里层 CMakeLists.txt

[参考资料4](#资料4)

```cmake
cmake_minimum_required(VERSION 3.16)
project(snpe_sample)

# 嵌套还有一层子目录生成动态库
add_subdirectory(fusioncore)

# 明确指定需要什么源文件
set(SOURCE_LIST
    3dnms.cpp
    CheckRuntime.cpp
    CreateUserBuffer.cpp
    LoadContainer.cpp
    LoadInputTensor.cpp
    PreprocessInput.cpp
    SaveOutputTensor.cpp
    SetBuilderOptions.cpp
    Util.cpp
    main.cpp
)
# 如果实在太多，不想一个个添加，可以用下面的命令
# aux_source_directory(<path> <variable>)
# aux_source_directory(. SOURCE_LIST)

# 交叉编译
if(CROSS_COMPILE)
    message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}.")
    message(STATUS "CROSS_COMPILE has set to be ${CROSS_COMPILE}, now start compiling for aarch64.")
    set(SNPE_ROOT $ENV{HOME}/Projects/pcrossing8155/snpe-1.63.0.3523)

    set(absl_DIR "/tmp/stage/lib/cmake/absl")

    set(protobuf_MODULE_COMPATIBLE TRUE)
    set(Protobuf_DIR "/tmp/stage/lib/cmake/protobuf")
    find_package(Protobuf CONFIG REQUIRED)
    message(STATUS "Using protobuf ${Protobuf_VERSION}")

    set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
    set(_REFLECTION gRPC::grpc++_reflection)

    set(gRPC_DIR "/tmp/stage/lib/cmake/grpc")
    find_package(gRPC CONFIG REQUIRED)
    message(STATUS "Using gRPC ${gRPC_VERSION}")
    set(_GRPC_GRPCPP gRPC::grpc++)

    include_directories(${PROJECT_SOURCE_DIR}
                        ${SNPE_ROOT}/include/zdl/
                        ${CMAKE_CURRENT_SOURCE_DIR}/include
                        /home/huangruixin/Projects/data-common-cpp-demo/include)

    link_directories(${SNPE_ROOT}/lib/aarch64-oe-linux-gcc8.2
                        /home/huangruixin/Projects/data-common-cpp-demo/build)


    add_executable(${PROJECT_NAME} ${SOURCE_LIST})

    target_link_libraries(${PROJECT_NAME}
            frame_viz
            ${_REFLECTION}
            ${_GRPC_GRPCPP}
            ${_PROTOBUF_LIBPROTOBUF}
            SNPE)

# 本地编译
else()
    set(protobuf_MODULE_COMPATIBLE TRUE) # 设置变量，没搞清楚哪一步用到了这个变量，猜测是ProtobufConfig.cmake 需要用
    find_package(Protobuf CONFIG REQUIRED) # 指名使用config模式寻找 Protobuf 包
    message(STATUS "Using protobuf ${Protobuf_VERSION}") # log 信息

    set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf) # 设置变量，库文件路径

    find_package(Threads REQUIRED)
    find_package(PkgConfig REQUIRED)

    pkg_check_modules(GRPCPP REQUIRED grpc++>=1.22.0) # 确认 .pc 文件是否存在  #TODO


    include_directories(
        ${PROJECT_SOURCE_DIR} # CMakeLists.txt 同级目录下的所有头文件
        ${GRPCPP_INCLUDE_DIRS} # find_package 后自动生成的变量：头文件所在目录
        ${CMAKE_SOURCE_DIR}/include/zdl/ # CMAKE_SOURCE_DIR=<path_to_CMakeLists.txt>
        ${Protobuf_INCLUDE_DIRS} # find_package 后自动生成的变量：头文件所在目录
        /root/data-common-cpp-demo/include) # 额外的库里面的头文件

    link_directories(
        ${CMAKE_SOURCE_DIR}/lib/aarch64-oe-linux-gcc8.2
        ${GRPCPP_LIBRARY_DIRS} # find_package 后自动生成的变量：库文件所在位置		#TODO
        ${Protobuf_LIBRARIES} # find_package 后自动生成的变量：库文件所在位置		#TODO
        /root/data-common-cpp-demo/build)

    add_executable(${PROJECT_NAME} 
        ${SOURCE_LIST})

    target_include_directories(${PROJECT_NAME}
        PRIVATE fusioncore/include/public) # !‼️! 多个target存在时考虑的问题，这条指令囊括进来的头文件只能 ${PROJECT_NAME} 这个target使用，其他目标用不了，做好”隔离“，而 include_directories 指令囊括进来的头文件所有target都能用

    target_link_libraries(${PROJECT_NAME} # 同上
        ${GRPCPP_LIBRARIES}
        ${_PROTOBUF_LIBPROTOBUF}
        fusioncore
        frame_viz
        SNPES)

endif()
```



<font color =red>发散</font>：如果项目内有很多可执行文件需要编译，分散在各个目录中。有时并不需要编译所有的可执行文件，或者只需要编译几个。那这时就可以用上 if-option 进行配合，给一个默认值，并且用 if 判断是否需要构建。

# 致谢

在这个小工作的成型过程中，特别感谢实习期间LTX和YYF大哥对我的帮助，以及LH和LYF大哥对我的指点。

# 参考资料

1. [grpc官方库的交叉编译脚本](https://github.com/grpc/grpc/blob/master/test/distrib/cpp/run_distrib_test_cmake_aarch64_cross.sh)
2. [grpc quick start](https://grpc.io/docs/languages/cpp/quickstart/)
3. [cmake教程](https://blog.csdn.net/qq_38410730/article/details/102477162)
4. <span id = 资料4>helloword样例: </span>
   1. [common](https://github.com/grpc/grpc/blob/master/examples/cpp/cmake/common.cmake)
   2. [helloworld](https://github.com/grpc/grpc/blob/master/examples/cpp/helloworld/CMakeLists.txt)