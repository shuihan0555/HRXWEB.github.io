---
title: libyuv入门Scale相关内联汇编代码理解
subtitle:
date: 2024-01-03 13:52:03 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: libyuv
show_tags: true
---

通过改写libyuv的scale部分代码，来理解libyuv的内联汇编代码。
<!--more-->

# [libyuv入门]Scale相关内联汇编代码理解

由于业务的关系，希望能进一步减少libyuv的 scale 和 color convert 的耗时。

背景：cpu支持armv8和aarch64

看看源码部分能做些什么。

目前scale部分的代码经过修改已经得到了验证，逻辑上没有问题。因此本文以对Scale相关的代码的修改作为libyuv入门的第一课。

## ScaleRowDown2Linear_NEON

经过如下层层分析，可以确认笔者写的调用libyuv::I422scale代码，会调用与此节标题相同的函数名：

1. 支持neon
2. scale之后的width大小是<font color =red>16字节对齐</font>的
3. 最近邻差值
4. 缩放倍数是2

具体的分析过程不进行展开，重点放在此函数的内联汇编代码：

```cpp
// Read 32x1 average down and write 16x1.
void ScaleRowDown2Linear_NEON(const uint8_t* src_ptr,
                              ptrdiff_t src_stride,
                              uint8_t* dst,
                              int dst_width) {
  (void)src_stride;
  asm volatile(
      "1:                                        \n"
      // load even pixels into v0, odd into v1
      "ld2         {v0.16b,v1.16b}, [%0], #32    \n"
      "subs        %w2, %w2, #16                 \n"  // 16 processed per loop
      "urhadd      v0.16b, v0.16b, v1.16b        \n"  // rounding half add
      "prfm        pldl1keep, [%0, 448]          \n"  // prefetch 7 lines ahead
      "st1         {v0.16b}, [%1], #16           \n"
      "b.gt        1b                            \n"
      : "+r"(src_ptr),   // %0
        "+r"(dst),       // %1
        "+r"(dst_width)  // %2
      :
      : "v0", "v1"  // Clobber List
  );
}
```

再进行分析之前，有一些前置的知识，笔者不打算详细描述，直接给出笔者参考的“轮子”，即参考资料<sup>1</sup>，否则要说明白的话就得长篇大论了。

在对前置的知识有一定的了解后，正式开始分析这段汇编代码：

```nasm
# 首先给一个lable，方便之后跳转
"1:                                        \n"
```

[arm asm guide--ld2](https://developer.arm.com/documentation/dui0801/l/A64-SIMD-Vector-Instructions/LD2--vector--multiple-structures---A64-?lang=en):

```nasm
# neon寄存器有v0-v31共32个
# 这里将地址%0处开始的32字节内容分别load寄存器v0，v1，并且%0自增32
# 参看手册，需要注意一个词：”de-interleaving“（解交织），
# 即这一条汇编指令，应该这么理解：
# 每次都取两个连续Byte，一个存储在v0，一个存储在v1
# 执行16次这个过程，32Byte就两两一组分成了16组，每一组的两个成员分别放在v0和v1
# 比如RGB的存储方式是RGBRGBRGB...可以通过LD3解交织将R，G，B分别存储在一个寄存器上。
"ld2         {v0.16b,v1.16b}, [%0], #32    \n"
```

[arm asm guide--subs](https://developer.arm.com/documentation/dui0801/l/A64-General-Instructions/SUBS--immediate---A64-?lang=en)

```nasm
# 每次处理16个字节，因此每次都给 %w2 这个寄存器减去16，通过它来控制循环的次数
# %w2 表示的是 dst_width， 因此当未处理的宽度变成0了之后，就跳出循环不会再执行了
"subs        %w2, %w2, #16                 \n"  // 16 processed per loop
...
# 当 %w2 - 16 > 0，跳转到1的位置继续执行
"b.gt        1b                            \n"
```

[arm asm guide--urhadd](https://developer.arm.com/documentation/dui0801/l/A64-SIMD-Vector-Instructions/URHADD--vector---A64-?lang=en)

```nasm
# v0和v1寄存器内的数据执行半精度的加法，并且将得到的值右移1bit，即除以2，之后将结果存到v0
# 因此ld2时是解交织的，所以相邻的两个元素分别对应存储在v0和v1，执行相加除2就是最近邻（的某一种形式）。
"urhadd      v0.16b, v0.16b, v1.16b        \n"  // rounding half add
```

```nasm
# 预取数据的命令，请参考资料1，这里不想展开，简单来说就是在448Byte的数据keep在cache中，增加缓存命令率
"prfm        pldl1keep, [%0, 448]          \n"  // prefetch 7 lines ahead
```

```nasm
# 将16字节的数据从v0拷贝到%1所指示的地址，即dst所指示的位置，然后自增16
"st1         {v0.16b}, [%1], #16           \n"
```

```nasm
# 三个 ":" 的解释在资料1中已经有详细的阐释了，没记住的读者再回去看一眼吧。
: "+r"(src_ptr),   // %0
        "+r"(dst),       // %1
        "+r"(dst_width)  // %2
:
: "v0", "v1"  // Clobber List
```

经过了这一系列分析，终于可以开始改动这段代码了，笔者自己写了一个32位对齐的版本，原理是一样的，具体的指令可以看arm的asm手册<sup>4</sup>。

## ScaleRowDown2LinearAlign32_NEON

```cpp
// Read 64x1 average down and write 32x1.
void ScaleRowDown2LinearAlign32_NEON(const uint8_t* src_ptr,
                              ptrdiff_t src_stride,
                              uint8_t* dst,
                              int dst_width) {
  // std::cout << "Align 32" << std::endl;
  (void)src_stride;
  asm volatile(
      "1:                                        \n"
      // load even pixels into v0,2 odd into v1,3
      "ld4         {v0.16b,v1.16b,v2.16b,v3.16b}, [%0], #64                                 \n"
      "subs        %w2, %w2, #32                                                            \n"
      "urhadd      v1.16b, v0.16b, v1.16b                                                   \n"  // rounding half add
      "urhadd      v2.16b, v2.16b, v3.16b                                                   \n"  // rounding half add
      "prfm        pldl1keep, [%0, 1792]                                                    \n"  // prefetch 28 lines ahead
      "st2         {v1.16b,v2.16b}, [%1], #32                                                      \n"
      "b.gt        1b                            \n"
      : "+r"(src_ptr),   // %0
        "+r"(dst),       // %1
        "+r"(dst_width)  // %2
      :
      : "v0", "v1", "v2", "v3"  // Clobber List
  );
}
```

补充一点：

Q：为什么两个urhadd，一个要保存到v1，一个要保存到v2。

A：查阅[armasm Reference Guide--st2](https://developer.arm.com/documentation/dui0802/b/A64-SIMD-Vector-Instructions/ST2--vector--multiple-structures-)的使用方法，获悉两个寄存器需要连续，所以为了连续性，在这前面做了这样的操作。<font color=red>但是要补充的是</font>，在[armasm User Guide--st2](https://developer.arm.com/documentation/dui0801/l/A64-SIMD-Vector-Instructions/ST2--vector--multiple-structures---A64-?lang=en)中并没有对寄存器提出连续的要求，笔者也没有做实验非连续行不行。

----

neon的一些基础知识可以看看资料<sup>5</sup>

# 参考资料

1. [SIMD3: NEON 内联汇编](https://no5-aaron-wu.github.io/2022/06/14/SIMD-3-NeonAssembly/)
2. [ARM系列 -- NEON](http://aijishu.com/a/1060000000381103)
3. [armv8指令集](https://blog.csdn.net/QiangLi_strong/article/details/81475227)
4. [Arm Compiler armasm User Guide](https://developer.arm.com/documentation/dui0801/l?lang=en)
5. [neon简介](https://dumphex.github.io/2020/04/09/NEON/)

