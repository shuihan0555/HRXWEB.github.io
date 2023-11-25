---
title: colcon-mixin 初解
subtitle:
date: 2023-11-25 09:36:03
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS
show_tags: true

---
<!--more-->
# colcon-mixin 初解

> <sup>1</sup>An extension for [colcon-core](https://github.com/colcon/colcon-core) to fetch and manage CLI mixins from repositories.

笔者大致理解的此扩展的开发原由：在使用colcon进行编译的时候有时会传入很多cmake-args，或者其他的args，并且还是固定每次都要传入，因此通过mixin提供一个快捷输入这一大串args的方式

<font color =purple>按照笔者对资料2所给的示例的观察，一定要是类似于-ctest-args [\* [\* …]]，--cmake-args [* [* …]]这种可以迭代传入多个参数的-XX-args才行</font>

具体有没有起作用，可以在`<workspace>/log/latest/OpenMPTest/command.log`文件中看看具体运行了什么命令。

## 如何使用<sup>2</sup>

1. 安装插件：`pip install colcon-mixin`

2. 添加mixin文件（这里只以本地方式举例，通过网页的方式可自行研究资料2）

   ```shell
   # 克隆官方示例仓库（利用现有的好轮子）
   $ git clone https://github.com/colcon/colcon-mixin-repository.git
   # 添加mixin源
   $ colcon mixin add default file://`pwd`/colcon-mixin-repository/index.yaml
   # 更新源
   $ colcon mixin update default
   ```

3. 使用

   ```shell
   # 查看可以用的 mixin 参数
   $ colcon mixin show
   # 使用选定的参数，如：mixin-test
   $ colcon build --mixin mixin-test
   $ colcon test --mixin mixin-test
   ....
   ```

## 如何添加自定义的mixin文件

1. 在刚才clone的库的根目录添加.mixin文件，要求符合yaml的语法
   ```shell
   # 举例 openmptest.mixin，文件名是因为笔者在利用openmp测试colcon的一个bug，所以看起来很无厘头
   
   {
   		# build 意味着只能用在 colcon build
       "build": {
       		# 注意openmp-off和openmp-on这二者也要按照alphabetic order
           "openmp-off": {
               "--cmake-clean-cache", # 请注意这个。另：若复制粘贴请删除此注释，yaml不支持行内注释
               "cmake-args": [
                   "-DCMAKE_VERBOSE_MAKEFILE=ON",
                   "-DCMAKE_TOOLCHAIN_FILE='/root/cross_compile/cmake-toolchains/generic_linux.cmake'"
               ]
           },
           "openmp-on": {
               "cmake-args": [
                   "-DCMAKE_VERBOSE_MAKEFILE=ON",
                   "-DCMAKE_TOOLCHAIN_FILE='/root/cross_compile/cmake-toolchains/generic_linux.cmake'",
                   "-DCOUT_INFO=ON"
               ]
           }
       }
   }
   ```

2. 按照字典顺序，即alphabetic order将文件名`openmptest.mixin`添加到`index.yaml`

   ```yaml
   mixin:
   	- asan.mixin
   	- ...
   	- ninja.mixin
   	- openmptest.mixin
   	- sccache.mixin
   	- ...
   ```

3. 检查`openmptest.mixin`的格式是否符合要求

   ```shell
   # 安装python依赖库
   $ pip install yamllint
   # 检查文件语法
   $ cd /path/to/colcon-mixin-repository && python lint.py
   ```

4. <font color = red>更新源：`colcon mixin update default`</font>，此时也会检查一些语法问题，有的话请根据提示修改。<font color =red>每次更新index.yaml 或者 .mixin 文件都需要update</font>

5. 测试是否可用

   1. 先看看添加的这两个“参数”

      ```shell
      $ colcon mixin show
      ...
      - openmp-off
        --cmake-clean-cache: None # 注意这个None
        cmake-args: ['-DCMAKE_VERBOSE_MAKEFILE=ON', "-DCMAKE_TOOLCHAIN_FILE='/root/cross_compile/cmake-toolchains/generic_linux.cmake'"]
      - openmp-on
        cmake-args: ['-DCMAKE_VERBOSE_MAKEFILE=ON', "-DCMAKE_TOOLCHAIN_FILE='/root/cross_compile/cmake-toolchains/generic_linux.cmake'", '-DCOUT_INFO=ON']
      ...
      ```

   2. 测试用`CMakeLists.txt`

      ```cmake
      project(OpenMPTest)
      set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
      if(CMAKE_TOOLCHAIN_FILE)
        message(AUTHOR_WARNING TOOLCHAINFILE: ${CMAKE_TOOLCHAIN_FILE})
      else()
        message(AUTHOR_WARNING TOOLCHAINFILE: ${CMAKE_TOOLCHAIN_FILE})
      endif()
      
      if(COUT_INFO)
        message(AUTHOR_WARNING COUT_INFO: ${COUT_INFO})
      else()
        message(AUTHOR_WARNING COUT_INFO: OFF)
      endif()
      find_package(OpenMP REQUIRED)
      find_package(backward_ros REQUIRED)
      ```

   3. 使用：`colcon build --mixin openmp-on`，可以发现log出来的信息为：

      ```plain text
      CMake Warning (dev) at CMakeLists.txt:6 (message):
        TOOLCHAINFILE:/root/cross_compile/cmake-toolchains/generic_linux.cmake
      This warning is for project developers.  Use -Wno-dev to suppress it.
      
      CMake Warning (dev) at CMakeLists.txt:12 (message):
        COUT_INFO:ON
      This warning is for project developers.  Use -Wno-dev to suppress it.
      ```

   4. 使用：`colcon build --mixin openmp-on`，log信息为：

      ```plain text
      colcon build --mixin openmp-off
      WARNING:colcon.colcon_mixin.mixin.mixin_argument:Mixin key '--cmake-clean-cache' is not a valid argument for 'build'
      Starting >>> OpenMPTest
      [2.667s] WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'OpenMPTest' because it has no 'install' target
      --- stderr: OpenMPTest                         
      CMake Warning (dev) at CMakeLists.txt:7 (message):
        TOOLCHAINFILE:/root/cross_compile/cmake-toolchains/generic_linux.cmake
      This warning is for project developers.  Use -Wno-dev to suppress it.
      
      CMake Warning (dev) at CMakeLists.txt:15 (message):
        COUT_INFO:OFF
      This warning is for project developers.  Use -Wno-dev to suppress it.
      ```

      可以看到虽然加了`--cmake-clean-cache`，并且build时是可以传入此参数的，但是不能通过Mixin传入

6. <font color =red>说明：笔者为什么在这里测试CMAKE_TOOLCHAIN_FILE 和自定义的COUT_INFO变量呢？</font>这是因为笔者发现直接使用colcon build 传入多个 --cmake-args的时候，会存在失效的问题。再具体点说就是：

   1. `colcon build  --cmake-args -DCMAKE_TOOLCAHIN_FILE=XXX --cmake-args -DCOUT_INFO=ON`，log出来的信息是`TOOLCHAINFILE`:，`COUT_INFO:ON`，直接把工具链文件的一系列设定都吃了，导致交叉编译失效！！！离了大谱了。
   2. `colcon build --mixin openmp-on` 可以发现`TOOLCHAINFILE:/root/cross_compile/cmake-toolchains/generic_linux.cmake` 没有失效！！！
   3. 另外这个和cmake本身没有关系，笔者也做了测试，具体的测试过程可以看[issue](https://github.com/colcon/colcon-core/issues/599)

# 参考资料

1. [colcon-mixin github](https://github.com/colcon/colcon-mixin)
2. [colcon-mixin-repo github](https://github.com/colcon/colcon-mixin-repository/tree/master)
3. [colcon mixin docs](https://colcon.readthedocs.io/en/released/reference/verb/mixin.html)