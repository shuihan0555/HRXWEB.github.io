---
title: ubuntu--vscode--ctrl+shift+e or f失效
subtitle:
date: 2023-12-12 04:53:23 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: Ubuntu VScode
show_tags: true
---
在ubuntu20.04 系统上安装了vscode，但是发现快捷键`ctrl+shift+e`和`ctrl+shift+f`失效了，这两个快捷键是用来打开资源管理器和全局搜索的，非常方便，但是现在失效了。
<!--more-->

# ubuntu--vscode--ctrl+shift+e or f失效

## 解决方法

点击ubuntu的右上角的语言选项，展开。依次点击 `Preferences` -> `Shortcuts`，找到 `Switch Traditional/Simplfied Chinese`，删除其快捷键。此时就可以使用 `ctrl+shift+f` 了。

至于 `ctrl+shift+e`<sup>1</sup>：在终端输入 `ibus-setup`，点击 `Emoji`，找到 `Emoji annotation`，删除其快捷键。

# 参考资料

1. [How can I get ctrl+shift+e to shortcut to work in vscode?](https://askubuntu.com/questions/1388083/how-can-i-get-ctrlshifte-to-shortcut-to-work-in-vscode)
