---
title: ROS轮询机制
subtitle: 
date: 2022-11-20 18:50:30
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: ROS
show_tags: true

---

# 消息队列

> 发送消息过快时，来不及处理，回调函数会存放在消息队列当中

## 单话题

显而易见，只存在一个消息队列

## 多话题

默认情况下，所有的回调函数共享同一个全局消息队列

> By default, **all callbacks get assigned into that global queue**, which is then processed by `ros::spin()` or one of the alternatives.

# ros::spin()

不断的轮询回调函数队列，有就立马处理，没有就<font color = red>阻塞</font>

# ros::spinOnce()

每执行一次就执行回调函数队列中所有的消息队列，然后继续向下执行，<font color = red>不阻塞</font>



# 参考资料

1. [ros::spin、ros::spinOnce 使用细节、区别](https://blog.csdn.net/weixin_40215443/article/details/103793316)
2. [ROS：回调函数处理与回调队列](https://blog.csdn.net/Azahaxia/article/details/113934774)
3. [ros wiki](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)