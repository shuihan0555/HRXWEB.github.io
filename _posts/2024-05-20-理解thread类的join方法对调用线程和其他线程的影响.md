---
title: 理解thread类的join方法对调用线程和其他线程的影响
subtitle:
date: 2024-05-20 01:35:55 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: thread C/C++
show_tags: true
---

如何处理 `vector<thread>` 中的线程？`join` 方法对调用线程和其他线程的影响是什么？这篇文章将解答这些问题。
<!--more-->

## join 的作用

`join` 方法的作用是等待线程结束。`join` 方法会阻塞调用线程，直到线程结束。

## 问题

考虑下面的代码：

```cpp
#include <iostream>
#include <thread>
#include <vector>

void foo() {
    std::cout << "foo" << std::endl;
}

int main() {
    std::vector<std::thread> threads;
    for (int i = 0; i < 10; ++i) {
        threads.push_back(std::thread(foo));
    }

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}
```

当对第一个线程调用 `join` 方法时，会发生什么？调用线程会阻塞，那么其他线程会怎么样？也会阻塞吗？

## 答案

当对第一个线程调用 `join` 方法时，调用线程会阻塞，直到第一个线程结束。其他线程不会受到影响，它们会继续执行。

### 验证

我们可以通过下面的代码验证这一点：

```cpp
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

std::mutex mtx;  // 互斥锁，用于保护共享变量的访问
std::atomic<int> sharedVariable(0);  // 原子类型的共享变量

void printAndIncrement()
{
    auto threadId = std::this_thread::get_id();  // 获取当前线程ID
    while (sharedVariable < 100)  // 检查共享变量的值是否小于目标值
    {
        std::lock_guard<std::mutex> lock(mtx);  // 加锁，保护共享变量的访问
        if (sharedVariable < 100)  // 再次检查共享变量的值是否小于目标值
        {
            std::cout << "Thread ID: " << threadId << ", Shared Variable: " << sharedVariable << std::endl;
            sharedVariable++;  // 递增共享变量
        }
    }
}

int main()
{
    std::vector<std::thread> threads;

    for (int i = 0; i < 10; i++)  // 创建10个线程
    {
        threads.emplace_back(printAndIncrement);
    }

    // 等待所有线程执行完毕
    for (auto& thread : threads)
    {
        thread.join();
    }

    return 0;
}
```

编译命令为：

```shell
g++ -std=c++11 -o main main.cpp -lpthread
```

从运行结果可以发现多个线程并发执行，其他线程并没有被阻塞。

## 如何理解？

`join` 方法只会阻塞调用线程，不会阻塞其他线程。因此，其他线程会继续执行。

如果调用 `join` 函数之前，线程就已经执行完了，那么就不会阻塞调用线程。
