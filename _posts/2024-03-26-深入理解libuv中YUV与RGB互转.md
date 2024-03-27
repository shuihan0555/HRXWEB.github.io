---
title: 深入理解libuv中YUV与RGB互转
subtitle:
date: 2024-03-26 13:03:34 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: libyuv
show_tags: true
---

通过测试的方式来理解libuv中YUV与RGB互转的过程，主要确定其使用的公式。
<!--more-->

# 以I420ToRGB24为例

```cpp
// libyuv/source/convert_argb.cc
// Convert I420 to RGB24.
LIBYUV_API
int I420ToRGB24(const uint8_t* src_y,
                int src_stride_y,
                const uint8_t* src_u,
                int src_stride_u,
                const uint8_t* src_v,
                int src_stride_v,
                uint8_t* dst_rgb24,
                int dst_stride_rgb24,
                int width,
                int height) {
  return I420ToRGB24Matrix(src_y, src_stride_y, src_u, src_stride_u, src_v,
                           src_stride_v, dst_rgb24, dst_stride_rgb24,
                           &kYuvI601Constants, width, height);
}
```

可以看到默认使用的是`kYuvI601Constants`，这个常量在`libyuv/include/libyuv/convert_argb.h`中声明如下：

```cpp
LIBYUV_API extern const struct YuvConstants kYuvI601Constants;   // BT.601
```

具体在哪里定义实在是找不到，但是从注释可以看出采用的是BT.601的标准。

BT.601的公式可以参考这篇博客<sup>1</sup>

> 接下来做实验验证一下

```cpp
/** BT.601
 * R = 1.164(Y - 16) + 1.596(V - 128)
 * G = 1.164(Y - 16) - 0.392(U - 128) - 0.813(V - 128)
 * B = 1.164(Y - 16) + 2.016(U - 128)
*/
void yuv2rgb()
{
    // h is 4, w is 2
    std::vector<uchar> y_data{17, 18, 19, 20, 21, 22, 23, 24};
    std::vector<uchar> u_data{120, 130, 240, 250};
    std::vector<uchar> v_data{130, 140, 130, 160};
    std::vector<uchar> rgb_data(8 * 3, 0);
    libyuv::I422ToRGB24(y_data.data(), 2,
                        u_data.data(), 1,
                        v_data.data(), 1,
                        rgb_data.data(), 6,
                        2, 4);
    cv::Mat img = cv::Mat(rgb_data, true).reshape(3, 4);
    std::cout << img << std::endl;
    std::cout << "---------------------" << std::endl;
    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    std::cout << "channels: ";
    std::copy(channels.begin(), channels.end(), std::ostream_iterator<cv::Mat>(std::cout, "\n"));
    std::cout << std::endl;

    std::cout << "B: ";
    for (int i = 0; i < 8; ++i)
    {
        int B = std::max(1.164 * (y_data[i] - 16) + 2.016 * (u_data[i/2] - 128), 0.);
        std::cout << B << " ";
    }
    std::cout << std::endl;
    std::cout << "G: ";
    for (int i = 0; i < 8; ++i)
    {
        int G = std::max(1.164 * (y_data[i] - 16) - 0.392 * (u_data[i/2] - 128) - 0.812 * (v_data[i/2] - 128), 0.);
        std::cout << G << " ";
    }
    std::cout << std::endl;
    std::cout << "R: ";
    for (int i = 0; i < 8; ++i)
    {
        int R = std::max(1.164 * (y_data[i] - 16) + 1.596 * (v_data[i/2] - 128), 0.);
        std::cout << R << " ";
    }
    std::cout << std::endl;
}

int main()
{
    yuv2rgb();
    return 0;
}
```

最终输出结果如下：

```plaintext
[  0,   3,   4,   0,   4,   6;
   7,   0,  23,   9,   0,  24;
 230,   0,   9, 231,   0,  10;
 252,   0,  59, 253,   0,  60]
---------------------
channels: [  0,   0;
   7,   9;
 230, 231;
 252, 253]
[  3,   4;
   0,   0;
   0,   0;
   0,   0]
[  4,   6;
  23,  24;
   9,  10;
  59,  60]

B: 0 0 7 8 231 232 254 255 
G: 2 3 0 0 0 0 0 0 
R: 4 5 22 23 9 10 59 60 
```

可以看到有些值会差1，应该是内部实现的问题，但是整体的计算过程是正确的。

说明libyuv中的YUV与RGB互转默认确实是使用的BT.601的标准。

# 参考资料

1. [BT601/BT709/BT2020 YUV2RGB RGB2YUV 公式](https://blog.csdn.net/m18612362926/article/details/127667954)