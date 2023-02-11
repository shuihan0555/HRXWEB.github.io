---
title: kitti2rosbag
subtitle: 用kitti点云数据做ros包
date: 2022-11-20 18:50:26
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS kitti ROSbag
show_tags: true

---

# kitti 数据转 rosbag

由于需要验证笔者写的 3D 点云数据转 range image 是否成功，由于项目组已经写好了 rosmsg 到 pcd 的内容。因此想将 kitti 的数据录成一个 bag 包，发布出去，然后在项目的代码里添加转 range image 模块，可以直接将项目已经转好的 pcd 作为 range image 模块的输入。

|--------------------|												 |------------ |				  |--------------|

| velodyne/XXX.bin | --------> rosbag --------> | Ptr --> pcd | --------> | range image |

|--------------------|												 |------------ |				  |--------------|

当然，想要直接验证的话，就是直接读取 kitii velodyne 数据 .bin 转成 pcd，就可以输入到 range image 中，不需要借助 ros 来回转换。

笔者目前是一窍不通的状态，先从用轮子开始

## build executable file

在 github 上面找 [轮子](https://github.com/AbnerCSZ/lidar2rosbag_KITTI)

```bash
# 建立工作空间 并且 进到 src 目录（src 存放最小单元--功能包）
$ mkdir ~/learning_ros/src && cd ~/learning_ros/src
# 拉取此项目作为功能包
$ git clone https://github.com/AbnerCSZ/lidar2rosbag_KITTI
# 回到工作空间根目录
$ cd ..
# 编译 release 版本，可以进行代码优化 # 如果这一步有问题，请看 “可能遇到的问题” 这一节
$ catkin_make -DCMAKE_BUILD_TYPE=Release
# 配置环境
$ source devel/setup.bash
```

## using

```bash
# 每次编译完或者切换了一个新窗口，需要重新配置环境变量
$ source devel/setup.bash
# 用法
$ rosrun lidar2rosbag lidar2rosbag <path_to_data> <name_of_bag>
# 实例 # 注意01目录后面的 / 一定不能少，否则会
# 报错！！！ ：segmentation fault (core dumped)
$ rosrun lidar2rosbag lidar2rosbag /dataset/sequences/01/ test

# 其中01文件夹下的内容为：
$ tree -L 1
.
|-- calib.txt
|-- labels
|-- poses.txt
|-- times.txt
|-- velodyne
# 其中 times.txt 文件 和 velodyne 目录是必须要的，其余的在此项目不会被用到
```

## 可能遇到的问题

### <font color =red>/usr/include/lz4.h:196:57: error: conflicting declaration ‘typedef struct LZ4_stream_t LZ4_stream_t’</font>

```bash
In file included from /opt/ros/melodic/include/roslz4/lz4s.h:38:0,
                 from /opt/ros/melodic/include/rosbag/stream.h:46,
                 from /opt/ros/melodic/include/rosbag/chunked_file.h:46,
                 from /opt/ros/melodic/include/rosbag/bag.h:41,
                 from /home/huangruixin/learning_ros/src/kitti2rosbag/src/kitti2rosbag.cpp:33:
/usr/include/lz4.h:196:57: error: conflicting declaration ‘typedef struct LZ4_stream_t LZ4_stream_t’
 typedef struct { long long table[LZ4_STREAMSIZE_U64]; } LZ4_stream_t;
                                                         ^~~~~~~~~~~~
In file included from /usr/include/flann/util/serialization.h:9:0,
                 from /usr/include/flann/util/matrix.h:35,
                 from /usr/include/flann/flann.hpp:41,
                 from /usr/include/pcl-1.8/pcl/kdtree/flann.h:50,
                 from /usr/include/pcl-1.8/pcl/kdtree/kdtree_flann.h:45,
                 from /usr/include/pcl-1.8/pcl/search/kdtree.h:44,
                 from /home/huangruixin/learning_ros/src/kitti2rosbag/src/kitti2rosbag.cpp:20:
/usr/include/flann/ext/lz4.h:196:57: note: previous declaration as ‘typedef struct LZ4_stream_t LZ4_stream_t’
 typedef struct { long long table[LZ4_STREAMSIZE_U64]; } LZ4_stream_t;
                                                         ^~~~~~~~~~~~
In file included from /opt/ros/melodic/include/roslz4/lz4s.h:38:0,
                 from /opt/ros/melodic/include/rosbag/stream.h:46,
                 from /opt/ros/melodic/include/rosbag/chunked_file.h:46,
                 from /opt/ros/melodic/include/rosbag/bag.h:41,
                 from /home/huangruixin/learning_ros/src/kitti2rosbag/src/kitti2rosbag.cpp:33:
/usr/include/lz4.h:249:72: error: conflicting declaration ‘typedef struct LZ4_streamDecode_t LZ4_streamDecode_t’
 typedef struct { unsigned long long table[LZ4_STREAMDECODESIZE_U64]; } LZ4_streamDecode_t;
                                                                        ^~~~~~~~~~~~~~~~~~
In file included from /usr/include/flann/util/serialization.h:9:0,
                 from /usr/include/flann/util/matrix.h:35,
                 from /usr/include/flann/flann.hpp:41,
                 from /usr/include/pcl-1.8/pcl/kdtree/flann.h:50,
                 from /usr/include/pcl-1.8/pcl/kdtree/kdtree_flann.h:45,
                 from /usr/include/pcl-1.8/pcl/search/kdtree.h:44,
                 from /home/huangruixin/learning_ros/src/kitti2rosbag/src/kitti2rosbag.cpp:20:
/usr/include/flann/ext/lz4.h:249:72: note: previous declaration as ‘typedef struct LZ4_streamDecode_t LZ4_streamDecode_t’
 typedef struct { unsigned long long table[LZ4_STREAMDECODESIZE_U64]; } LZ4_streamDecode_t;
                                                                        ^~~~~~~~~~~~~~~~~~
In file included from /usr/include/pcl-1.8/pcl/sample_consensus/sac_model.h:52:0,
                 from /usr/include/pcl-1.8/pcl/sample_consensus/sac.h:45,
                 from /usr/include/pcl-1.8/pcl/segmentation/sac_segmentation.h:49,
                 from /home/huangruixin/learning_ros/src/kitti2rosbag/src/kitti2rosbag.cpp:23:
/usr/include/pcl-1.8/pcl/sample_consensus/model_types.h: In function ‘void __static_initialization_and_destruction_0(int, int)’:
/usr/include/pcl-1.8/pcl/sample_consensus/model_types.h:99:3: warning: ‘pcl::SAC_SAMPLE_SIZE’ is deprecated: This map is deprecated and is kept only to prevent breaking existing user code. Starting from PCL 1.8.0 model sample size is a protected member of the SampleConsensusModel class [-Wdeprecated-declarations]
   SAC_SAMPLE_SIZE (sample_size_pairs, sample_size_pairs + sizeof (sample_size_pairs) / sizeof (SampleSizeModel));
   ^~~~~~~~~~~~~~~
/usr/include/pcl-1.8/pcl/sample_consensus/model_types.h:99:3: note: declared here
kitti2rosbag/CMakeFiles/kitti2rosbag.dir/build.make:62: recipe for target 'kitti2rosbag/CMakeFiles/kitti2rosbag.dir/src/kitti2rosbag.cpp.o' failed
make[2]: *** [kitti2rosbag/CMakeFiles/kitti2rosbag.dir/src/kitti2rosbag.cpp.o] Error 1
CMakeFiles/Makefile2:480: recipe for target 'kitti2rosbag/CMakeFiles/kitti2rosbag.dir/all' failed
make[1]: *** [kitti2rosbag/CMakeFiles/kitti2rosbag.dir/all] Error 2
Makefile:140: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j8 -l8" failed
```

```bash
# 备份 /usr/include/flann/ext/lz4.h 和 /usr/include/flann/ext/lz4hc.h
$ sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
$ sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak

# 将 /usr/include/flann/ext/lz4.h 指向 /usr/include/lz4.h
$ sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
# 同上
$ sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
```

### <font color = red>segmentation fault (core dumped)</font>

或者中文：段错误（核心已转储）

笔者遇到的一个原因主要是：

[源代码的这几行](https://github.com/AbnerCSZ/lidar2rosbag_KITTI/blob/901ecd26bb04db3d645e521c252ea66d1be7c29b/src/lidar2rosbag.cpp#L181-L183)是这么写的：

```cpp
std::string input_dir = argv[1];
std::string output_dir = argv[2];
std::string bin_path = input_dir + "velodyne/";//"/data/KITTI/dataset/sequences/04/velodyne/";
```

特别注意第三行， 因此 argv[1] 一定要以 / 结尾，即

```bash
# 会报错
$ rosrun lidar2rosbag lidar2rosbag /dataset/sequences/01 test
# 不会报错
$ rosrun lidar2rosbag lidar2rosbag /dataset/sequences/01/ test
```





