---
title: 使用submodule管理仓库
subtitle:
date: 2023-09-25 23:44:48
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: git
show_tags: true

---

submodule使用方法初探

<!--more-->

# 使用submodule管理仓库

## what is submodule

简单来说就是在主仓库中嵌入另一个仓库作为子仓库的管理方式。主仓库能够跟踪和管理子仓库的版本。

## why need submodule

发明它最直观的理由可以使得多个库之间解耦，方便代码管理，具体更过的功能笔者并没有切实体会，以下只列出目前在做项目的过程中最切实的一个使用场景，后续更多使用场景未来再做补充

1. 当多个部门进行协调工作时，各自的仓库都是分开的。若另一部门开发的代码是本部门所开发代码的依赖库，且其经常发生变化时，尤其是在项目初期，一些变化很容易导致不兼容的问题。因此在开发本部门代码的过程中，将另一部门的代码库作为submodule来进行管理，同时管理主仓库和子仓库，保证其对应版本的代码能够顺利跑通。后续可以无感进行开发工作，因为submodule会记录下来此时子仓库的commit ID，子仓库的远程更新并不影响下次clone此主仓库时依赖的子仓库版本。

## 如何添加submodule

```shell
$ git submodule add [-b <branch>] <repository_url> [<path/to/directory>]
# 例如：
$ cd /path/to/superRepo
$ git submodule add -b dev https://github.com/example/my-submodule.git thirdparty
# 此时有一个在 /path/to/superRepo 目录下，有一个 thirdparty 目录，是远程子仓库的本地 workspace
```

## 如何查看主仓库引用的submodule版本

```shell
$ git submodule <status> 		# status 可加可不加
# 此时console会打印出 submodule name 和其对应的commit ID
```

## 如何clone带有submodule的仓库

```shell
# 方法1
$ git clone --recurse-submodules <repository_url

# 方法2.1
$ git submodule init				# 读取 .gitsubmodule 文件中的相关配置信息
$ git submoule update 			# 拉取子模块的代码并checkout到正确的版本

# 方法2.2--将2.1的方法合并成一条命令
$ git submodule update --init <--recursive> # --recursive 参数可以保证子仓库的子仓库也一并拉取
```

## 如何切换带有submodule的仓库的分支

可能主仓库的两个分支索引的submodule版本不一样，所以当切换完分支之后：

```shell
# 例如一个实际的例子，可以从输出看到提示我们submodule被修改了
$ git checkout main
M	test-submodule
Switched to branch 'main'
Your branch is up to date with 'origin/main'.

# 此时我们需要做的事情就是更新submodule，更新引用的版本
$ git submodule update
```

## 更新submodule/主仓库

```shell
# 只更新子仓库
$ git submodule update --remote <submodule-name> # 更新名为 submodule-name 的 submodule 为远程项目的最新版本，不加名字则更新所有的submodule

# 同时更新 submodule 和主仓库
$ git pull --recurse-submodules

# 补充：对所有子仓库执行同样的操作：
$ git submodule foreach '<arbitrary-command-to-run>'
# 如：
$ git submodule foreach 'git checkout main'
```

## 为什么在主仓库git pull之后不更新submodule

git pull 默认会递归fetch，但只有主仓库的代码会进行merge

但是 git pull --recurse-submodules 会merge submodule。

## 一些有用的设置

### 每次push代码之前检查submodule的更改有没有一并push

为什么这么做？因为在你本地能跑通的版本，才是主仓库和submodule对应的版本，所以要一起进行更新才能保证兼容。

```shell
# 方法1--每次push代码时都执行
$ git push --recurse-submodules=
				# 1. check: 检查 submodule 是否有提交未推送, 如果有, 则使本次提交失败
				# 2. on-demand: 先推送 submodule 的更新, 然后推送主项目的更新 (如果 submodule 推送失败, 那么推送任务直接终止)
				# 3. while: 所有的 submodule 会被依次推送到远端, 但是 superproject 将不会被推送
				# 4. no: 与 while 相反, 只推送 superproject, 不推送其他 submodule

# 方法2--设置模式永远都为check
$ git config --global push.recurseSubmodules check
```

## 为什么每次 update 后 submodule 的 HEAD 状态变为了 detached?

根据资料<sup>2</sup>的说法，在一个主项目中引入了 Submodule 其实 Git 做了3件事情：

- 记录引用的仓库
- 记录主项目中Submodules的目录位置
- 记录引用Submodule的commit id

执行 git submodule update 的时候 git 就根据 gitlink 获取submodule 的 commit id，最后获取 submodule 的文件，所以 clone 之后不在任何分支上。

## 什么情况下主仓库会更新对submodule引用的版本

1. git submodule update --remote ?
2. git submodule update ?

# 实际测试记录

```shell
# 准备主仓库
$ cd /path/you/want
$ mkdir -p userA/test-super
$ cd userA/test-super
$ git init
$ echo "# test-super" >> README.md
$ git commit -m "init repo"
$ git remote add origin git@github.com:HRXWEB/test-super.git
$ git push -u origin main

# 准备子仓库
$ cd /tmp
$ mkdir test-submodule && cd test-submodule
$ echo "# test-submodule" >> README.md
$ git add .
$ git commit -m "init repo"
$ git remote add origin git@github.com:HRXWEB/test-submodule.git

# 回到主仓库
$ cd /path/you/want/userA/test-super
$ git submodule add git@github.com:HRXWEB/test-submodule.git test-submodule

# 查看子仓库的状态
$ git submodule
5d07c13b79b255cbfafacf58d78a83342e6db299 test-submodule (heads/main)
# 不知道为什么没有提示detach，而且使用下面的命令发现子仓库正是在main分支下
$ cd test-submoule && git branch
* main

# 更改子仓库
$ cd test-submodule
$ touch a.txt
$ git add .
$ git commit -m "add a.txt for test"
[main f64700c] add a.txt for test    # 记住 f64700c
 1 file changed, 0 insertions(+), 0 deletions(-)
 create mode 100644 a.txt

# 回到主仓库
$ cd ..
# 使用命令可以发现主仓库只记录子仓库有更改，不记录具体修改
$ git status
On branch main
Your branch is up to date with 'origin/main'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   test-submodule (new commits)

no changes added to commit (use "git add" and/or "git commit -a")
# 查看子仓库的状态
## + 表示子仓库进行了修改提交
## f64700 开头和之前提交子仓库的修改显示的哈希值一样
$ git submodule
+f64700c85d2e45bc97a5990926ed9b88ac75a141 test-submodule (heads/main)


# 依然在主仓库
$ git push
Everything up-to-date
```

> 由于上面的状态很重要，作为初步接触submodule的笔者来说，需要单独拿出来说一说。

由log的信息可以得知主仓库并不会自动递归推送(push)子仓库的更改，因此主仓库记录的子仓库版本还是5d07c13这个版本。接下来做一些测试看什么时候主仓库更改了对子仓库版本的引用

## 测试1

```shell
# 修改主仓库并提交推动
$ touch a.txt
$ git add .
$ git commit -m "add a.txt for test"
[main 4226112] add a.txt for test
 2 files changed, 1 insertion(+), 1 deletion(-)
 create mode 100644 a.txt
# 再次查看主仓库的status，发现更改都已经提交了，包括子仓库！
$ git status
On branch main
Your branch is ahead of 'origin/main' by 1 commit.
  (use "git push" to publish your local commits)

nothing to commit, working tree clean
$ git push
Enumerating objects: 4, done.
Counting objects: 100% (4/4), done.
Delta compression using up to 8 threads
Compressing objects: 100% (2/2), done.
Writing objects: 100% (3/3), 344 bytes | 344.00 KiB/s, done.
Total 3 (delta 0), reused 0 (delta 0), pack-reused 0
To github.com:HRXWEB/test-super.git
   39ed5ca..4226112  main -> main
$ git submodule
 f64700c85d2e45bc97a5990926ed9b88ac75a141 test-submodule (heads/main)
```

> 看了很多文章，笔者以为这里不会更新对submodule的引用，所以以这段文字做个隔断，达到强调的作用

### 测试1总结

可以看见 `f64700c85` 前面没有 `+` 了，然后去github验证了一番，发现真的更新了对子仓库的引用，说明git push的情况分两类：

1. 主仓库没有更改，那么即使子仓库更改了也不会递归push代码，不改变远程主仓库对子仓库的引用版本。（正常，毕竟远程子仓库都没有新的修改内容）
2. 主仓库更改了，并且commit了submodule的修改后，进行push，此时远程主仓库对子仓库的引用。（<font color =red>但需要特别注意的是，虽然远程子仓库后面的@commitID改变了，但是实际点进去会发现并不存在这一版本的远程子仓库，详见测试4</font>）

<font color = purple>一句话：子仓库commit，主仓库不修改，在主仓库git push不更新远程引用</font>

## 测试2

```shell
# 回到子仓库
$ cd test-submodule
$ touch b.txt
$ git add .
$ git commit -m "add b.txt for test"
[main 7999075] add b.txt for test
 1 file changed, 0 insertions(+), 0 deletions(-)
 create mode 100644 b.txt
$ git push

# 回到主仓库
$ cd ..
$ git submodule
+79990752ee85b6a3d87f4ba5935442f58383a567 test-submodule (heads/main)
# 可以得知远程肯定没有更新对子仓库的引用
$ git push && git submodule
Everything up-to-date
+79990752ee85b6a3d87f4ba5935442f58383a567 test-submodule (heads/main)
# 可以发现既没有办法更新主仓库，也没法更新远程仓库对子仓库的引用版本
$ git status
git status
On branch main
Your branch is up to date with 'origin/main'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   test-submodule (new commits)

no changes added to commit (use "git add" and/or "git commit -a")
```

> 再次加分隔以示重点

可以发现主仓库此时记录了子仓库有更改，因此可以进行添加/提交/推送

```shell
$ git add .
# 试试看下面这两条命令
$ git submodule update
$ git submodule
 79990752ee85b6a3d87f4ba5935442f58383a567 test-submodule (heads/main)
 # 上面的log说明本地引用的版本进行了更新
$ git commit -m "update submodule"
$ git push 
# 去到github发现远程主仓库对子仓库的引用版本进行了更新，更新到了79990752
```

### 测试2结论

1. 子仓库即使自己推送了也不会更改远程主仓库对其引用的版本。并且子仓库即使推送了，回到主仓库，仍然发现子仓库时modified状态，可以进行add/commit/push操作，按顺序操作完后远程引用的就是新版本子仓库

<font color = purple>一句话：子仓库push，主仓库不修改，在主仓库git push不更新远程引用</font>

> 测试1/2再更抽象，即，子仓库的add/commit/push只改变本身和远程子仓库，主仓库在其改变后，无论对子仓库做什么操作，status显示的submodule的状态都是modified，因此只要当作普通的文件/目录来看待，再次在主仓库进行add/commit/push即可。

## 测试3

```shell
# 回到子仓库
$ cd test-submodule
$ touch c.txt
$ git add .
$ git commit -m "add c.txt for test"
[main 24fa3f7] add c.txt for test
 1 file changed, 0 insertions(+), 0 deletions(-)
 create mode 100644 c.txt


# 回到主仓库
$ cd ..
$ git push --recurse-submodules=check
$ git submodule
+24fa3f7aa51e2396ddc849509ecf6bd49af0bd74 test-submodule (heads/main)
# 去往主仓库和子仓库都执行
$ git status
# 可以发现push其实没起作用，主仓库没有add是肯定的，子仓库也没有推送，即子仓库在目前的操作并不能单独推送
## 另外on-deman等配置也试过了，就是推送不上去的。
```

不知道是不是git版本的原因，一番实操下来，总感觉和网上的文章的说法并不一样，如果主仓库真的是以普通目录来对待子仓库的话，肯定需要在主仓库中进行add/commit/push操作才能将子仓库的修改推送到远程主仓库，并更新对子仓库的引用。但是正如测试1红色文字所述，push时需要注意，以测试4来具体shuoming

## 测试4

```shell
# 经过测试3后，目前的状态是
# 1. 主仓库还没有add submodule 的 modify
# 2. 子仓库已经commit，但是还未push，ahead of origin/main by 1 commit

# 在主仓库进行add/commit/push
$ git add .
$ git commit -m "update c.txt of submodule"
$ git push 
# 去往GitHub可以发现，远程主仓库对子仓库的引用版本已经更新了，但是点击却发现404，说明此时并没有push submodule 的更改

# 去往子仓库
$ cd test-submodule
$ git status
On branch main
Your branch is ahead of 'origin/main' by 1 commit.
  (use "git push" to publish your local commits)

nothing to commit, working tree clean
# 可以看到，依然是ahead of，说明子仓库确实没有被提交到远程
```

> 经过这一番测试，就能够领悟为什么之前要设置recurse-submoule=check。因为这样才能够保证远程主仓库引用的子仓库版本是存在于GitHub上的。不然只能圈地自萌，只有本地版本能跑通。

```shell
```

# 删除submodule

```shell
$ git submodule deinit -f --all # 全部删除
$ git submodule deinit -f -- test-submodule
$ rm -rf .git/modules/test-submodule
$ git rm -f test-submodule
# 查看当前状态
$ git status
On branch main
Your branch is up to date with 'origin/main'.

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
	modified:   .gitmodules
	deleted:    test-submodule
# 执行 add/commit/push 即可
```

# 参考资料

1. [Git Submodule 使用](https://hanleylee.com/articles/usage-of-git-submodule/)
1. [手把手教你git submoudule](https://github.com/xhlwill/blog/issues/30)