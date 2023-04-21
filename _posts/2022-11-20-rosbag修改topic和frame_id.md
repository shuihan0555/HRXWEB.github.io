---
title: ROSbag修改topic和frame_id
subtitle: 
date: 2022-11-20 18:50:35
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS ROSbag
show_tags: true

---

topic 借助 rosbag play 和 record 进行修改，而 frame_id 借助 bag_tools 进行修改。
<!--more-->
# rosbag 修改 topic 和 frame_id

## 查看 bag info

```shell
# 查看bag记录的topic等信息
$ rosbag info <bag_name>
>>>
path:		
version:		
duration:		
start:		
end:		
size:				3.0GB
message:		801
compression:
types:			sensor_msgs/PointCloud2
topics:			/calib_cloud	801msgs		: sensor_msgs/PointCloud2
# 查看某个topic的frame_id
# 思路：播放bag，查看topic的信息
$ rosbag play <bag_name> --clock -l
$ rostopic echo <topic_name> | grep frame_id
```

## 修改 topic

思路：播放 bag，并且将需要修改的 topic 的名字替换成想要的名字，并且通过 rosbag record 记录下来

```shell
$ rosbag record -O <new_bag_name.bag> <desired_topic_name>
# 加 -r 0.3 降低播放速度至30%，以防漏帧
$ rosbag play <bag_name> --clock -r 0.3 <origin_topic_name>:=<desired_topic_name>
```

## 修改某个 topic 的 frame_id

借助 ros 官方提供的 bag_tools 它被包含在 srv_tools 中，去官网参看[安装srv_tools](http://wiki.ros.org/srv_tools)教程

其中修改 frame_id 的说明在[此处](http://wiki.ros.org/bag_tools#change_frame_id.py)

```shell
# 安装 srv_tools
$ cd catkin_ws/src
$ mkdir srv_tools
$ cd srv_tools
$ git clone https://github.com/srv/srv_tools.git .
$ cd ../..
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic # install dependencies
$ catkin_make

# 如果希望可以无论在哪个终端都能使用 bag_tools，就直接按照 bag_tools 的教程
$ cd catkin_ws
$ catkin_make install --pkg bag_tools

# 如果希望自己激活环境变量，暂时在某个终端起作用
$ cd catkin_ws
$ source devel/setup.bash


# 使用 bag_tools
$ rosrun bag_tools change_frame_id.py -t <origin_topic_name> -f <desired_frame_id> -i <input_bag_name> -o <output_bag_name>
```

>change_frame_id 脚本使用方法：
>
>usage: change_frame_id.py [-h] -o OUTPUT_BAGFILE -i INPUT_BAGFILE -f FRAME_ID
>                     -t TOPIC [TOPIC ...]
>     
>reate a new bagfile from an existing one replacing the frame id of requested
>topics.
>
>optional arguments:
>-h, --help            show this help message and exit
>  -o OUTPUT_BAGFILE     output bagfile
>  -i INPUT_BAGFILE      input bagfile
>  -f FRAME_ID           desired frame_id name in the topics
>  -t TOPIC [TOPIC ...]  topic(s) to change

### rosdep 未安装

```shell
# 正常安装
$ sudo rosdep init 
$ rosdep update
```

<font color = red>一般情况下都会失败，笔者挂了🪜网址能打开也失败了</font>

<font color = blue>那就手动复现命令的效果</font>

```shell
# sudo rosdep init 就是下载了一个文件到一个目录下，既如此，就手动创建文件，并且填充内容
$ mkdir -p /etc/ros/rosdep/sources.list.d
$ vim 20-default.list
>>>
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
>>>

# 接下来笔者直接运行 update 就没问题了
$ rosdep update
# 若还是无法顺利运行，参考这篇博客 https://blog.csdn.net/qq_30267617/article/details/115028689
```

# 参考资料

1. [改变ros bag 中消息的frame_id 和话题名](https://blog.csdn.net/qq_30460905/article/details/116902697)
2. [srv_tools](http://wiki.ros.org/srv_tools)
3. [ROS安装中sudo rosdep init和rosdep update失败的终极解决方法（最新版本）](https://blog.csdn.net/qq_30267617/article/details/115028689)