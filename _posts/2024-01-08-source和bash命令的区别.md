---
title: source和bash命令的区别
subtitle:
date: 2024-01-08 13:55:03 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: shell
show_tags: true
---

source 和 bash 命令执行shell脚本的区别是什么？
<!--more-->

source 和 bash 是两个不同的命令，用于在 Bash shell 中执行脚本文件。

source 命令：

1. source 命令用于在<font color =red>当前</font>的 Bash shell 环境中执行指定的脚本文件。也可以使用点号 . 来代替 source 命令。
2.， source 命令会将指定的脚本文件中的命令和环境变量直接加载到当前的 shell 中，使得其中的变量和函数可以在当前的 shell 环境中生效。
通常用于执行设置环境变量、函数和别名等的脚本文件，以确保所做的更改对当前的 shell 会话立即生效。

bash 命令：

1. bash 命令用于启动一个<font color = blue>新的 Bash shell 进程</font>，并执行指定的脚本文件。
2. bash 命令会创建一个<font color = blue>新的子 shell 进程</font>，该子进程会加载并执行指定的脚本文件中的命令。
3. 通常用于在新的 shell 环境中运行脚本文件，而不会影响当前的 shell 环境。

主要区别：

1. source 命令会直接在当前的 shell 环境中加载和执行脚本文件中的命令和环境变量，而 bash 命令会启动一个新的子 shell 进程来执行脚本文件。
2. source 命令可以使得脚本文件中的变量和函数在当前的 shell 环境中生效，而 bash 命令执行的脚本文件中的变量和函数仅在新的子 shell 环境中生效。
3. source 命令没有创建新的进程，而 bash 命令会创建一个新的子 shell 进程。

需要注意的是，source 命令是 Bash 的内置命令，而 bash 命令是用于启动 Bash shell 解释器的可执行文件。因此，source 命令仅在 Bash 环境中可用，而 bash 命令可以在任何支持 Bash 的环境中使用。
