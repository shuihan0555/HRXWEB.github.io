---
title: 安装python-pcl
subtitle: 
date: 2022-12-01 18:23:34
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: PCL python-pcl
show_tags: true

---

# Ubuntu16

```shell
# 首先需要安装pcl库
$ sudo apt-get install libpcl-dev pcl-tools # ubuntu16.04上安装的是pcl-1.7版本
# 安装python-pcl库
$ pip install python-pcl # 直接安装的是0.3.0a1版本，和pcl-1.7兼容
```

# Ubuntu18

```shell
# 首先需要安装pcl库
$ sudo apt-get install libpcl-dev pcl-tools # ubuntu18.04上安装的是pcl-1.8版本
```

由于默认 pip 安装的版本是 0.3.0a1 只支持 pcl-1.7，所以安装完后运行 **import pcl** 会出现问题：<font color = red>"ImportError: libpcl_keypoints.so.1.7: cannot open shared object file: No such file or directory"</font>

因此需要通过源码安装

## 源码安装python-pcl 0.3.0rc1 版本

```shell
$ git clone https://github.com/strawlab/python-pcl.git
# 可以翻看 setup.py 文件可以看到版本名称被指定为 0.3.0rc1
```

> 由于`libpcl-dev`默认的`vtk`依赖是6.3，而`python-pcl`中的`setup.py`文件里的`vtk`版本是7.0，所以需要修改`setup.py`文件`726`行`vtk_version = '7.0'`为`vtk_version = '6.3'`，不然会出现```cannot find -lvtkalglib-7.0``的错误。
>
> 由于`vtk-6.3`依赖库要少于`vtk-7.0`， 因此需要删除752行的多余的依赖库, 删掉`'vtkXXXX-' + vtk_version`即可，其中`VtkXXXX`有以下库：
>
> vtkexpat
> vtkfreetype
> vtkgl2ps
> vtkhdf5
> vtkhdf5_hl
> vtkjpeg
> vtkjsoncpp
> vtklibxml2
> vtkNetCDF
> vtkNetCDF_cxx
> vtkoggtheora
> vtkpng
> vtkproj4
> vtksqlite
> vtktiff
> vtkzlib

```shell
$ cd python-pcl
$ python setup.py build_ext -i
$ python setup.py install
# 显示Finished processing dependencies for python-pcl==0.3.0rc1即为安装成功。
```

<font color =red>Q：安装路径在哪？</font>

A：安装在当前 python 路径的包中。例如激活了虚拟环境就装在虚拟环境的python包中。

Q：<font color =red>如何指定安装路径？</font>

A：[各类python包的安装方法及设置安装路径](https://blog.csdn.net/sowhatgavin/article/details/81912541)

# 参考资料

1. [Ubuntu 18.04安装python-pcl 解决ImportError: libpcl_keypoints.so.1.7问题（更新Ubuntu20）](https://blog.csdn.net/zsssrs/article/details/108492750)
2. [各类python包的安装方法及设置安装路径](https://blog.csdn.net/sowhatgavin/article/details/81912541)

