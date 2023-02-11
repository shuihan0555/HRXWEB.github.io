---
title: Rviz远程显示
subtitle: 
date: 2022-11-20 18:50:40
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS Rviz
show_tags: true

---

# Rviz在本地显示远程的topic

## 确定主/从机ip、用户名、主机名

主机：ros机器人

从机：本地的电脑

```shell
# 主机ip
$ ifconfig
# 用户名
$ whoami
# 主机名
$ hostname
```

## 修改bashrc

```shell
$ vim ~/.bashrc
# 在最后面加入如下两行
export ROS_MASTER_URI=http://<主机ip>:11311
export ROS_HOSTNAME=<从机ip>
# 至于端口号为什么是11311，可以自行在主机运行 roscore 命令
# 可以看到输出包含 ROS_MASTER_URI=http://<ip>:<port> 字眼
# 因此可以自行在主机中定义此环境变量，换一个你喜欢的端口
$ source ~/.bashrc
```

##  修改hosts文件

```shell
$ sudo vim /etc/hosts
# 在中间添加一行
<主机ip> <主机名>
```

