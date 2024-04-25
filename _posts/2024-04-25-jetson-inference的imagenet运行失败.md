---
title: Jetson Inference的imagenet运行报错
subtitle:
date: 2024-04-24 16:22:55 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: OrinAGX Nvidia Jetson
show_tags: true
---

<!--more-->



```shell
$ ./imagenet images/orange_0.jpg images/test/output_0.jpg
...
[OpenGL] failed to open X11 server connection.
[OpenGL] failed to create X11 Window.
...
```

nv的forums上也有讨论这个[问题](https://forums.developer.nvidia.com/t/video-output-does-not-show-up-because-opengl-failed-to-create-x11-window-imagenet-camera-failed-to-create-opengl-display/68681/6)

有人给出了[解决方案](https://forums.developer.nvidia.com/t/video-output-does-not-show-up-because-opengl-failed-to-create-x11-window-imagenet-camera-failed-to-create-opengl-display/68681/7?u=rxhuang1014)

注意上面有人说 `export DISPLAY=:0` 也可以解决问题，但是这个方法不适用于 Orin。解决方案说的是 TX2 是 0， Xavier 是 1。然后我试了一下，Orin 是 1。具体的原理目前不太清楚

```shell
$ export DISPLAY=:1
$ ./imagenet images/orange_0.jpg images/test/output_0.jpg
```

成功～

# 参考资料

1. [nvidia forums \[OpenGL\] failed to create X11 Window](https://forums.developer.nvidia.com/t/video-output-does-not-show-up-because-opengl-failed-to-create-x11-window-imagenet-camera-failed-to-create-opengl-display/68681/1)