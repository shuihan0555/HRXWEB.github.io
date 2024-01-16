---
title: 网页可以打开但是终端添加了代理也无法ping通github解决方法
subtitle:
date: 2024-01-16 15:43:01 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: git
show_tags: true
---

<!--more-->

# Github网页可以打开但是终端添加了代理也无法ping通github解决方法

状态：可以正常打开github网页，终端了添加了代理，但是无法ping通github.com

解决方法：

1. 访问https://github.com.ipaddress.com/www.github.com 获取目前github.com实际ip
2. 在Mac/Linux终端执行如下命令
    
    ```shell
    $ sudo echo -e "<ip/of/github.com> github.com\n199.232.5.194 github.global.ssl.fastly.net\n54.231.114.219 github-cloud.s3.amazonaws.com\n" >> /etc/hosts
    ```
3. 重新打开终端，即可正常访问github.com