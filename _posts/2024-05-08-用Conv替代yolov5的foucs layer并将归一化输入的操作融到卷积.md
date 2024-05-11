---
title: 用Conv替代yolov5的foucs layer并将归一化输入的操作融到卷积
subtitle:
date: 2024-05-08 15:04:13 +0800
lang: zh
author: Ricky Yel
show_edit_on_github: true
tags: onnx yolov5 ModelDeploy
show_tags: true
---

yolov5的foucs layer 可以用Conv替代，而且将归一化输入的操作融到卷积中，这样可以减少总体计算量，利于模型部署。
<!--more-->

# yolov5的foucs layer

```python
class Focus(nn.Module):
    # Focus wh information into c-space
    def __init__(self, c1, c2, k=1):
        super(Focus, self).__init__()
        self.conv = Conv(c1 * 4, c2, k, 1)

    def forward(self, x):  # x(b,c,w,h) -> y(b,4c,w/2,h/2)
        return self.conv(torch.cat([x[..., ::2, ::2],
                                    x[..., 1::2, ::2],
                                    x[..., ::2, 1::2],
                                    x[..., 1::2, 1::2]], 1))
```

效果为（深度换空间）：

```plaintext
transforms

[[[[ 0,  1,  2,  3],
   [ 4,  5,  6,  7],
   [ 8,  9, 10, 11],
   [12, 13, 14, 15]]]]

to 

[[[[ 0,  2],
   [ 8, 10]],

  [[ 4,  6],
  [12, 14]],

  [[ 1,  3],
   [9, 11]],

  [[5,  7],
  [13, 15]]]]
```

# 用Conv替代foucs layer

对于上面具体的例子，很容易就能想到可以用2\*2的卷积替代这个过程。

bias 很容易想到是0，因为这个过程只是一个换位操作，不需要额外的偏置。

而 weight 却比较困难，不过很容易想到的是每个卷积核只有一个位置是1.0，其他位置是0，这样就能实现上面的效果，问题就是如何确定值为1.0的位置

生成weight的代码如下，它适用于任何的step（不仅限于yolov5实现的2）：

```python
def pixel_unshuffle_to_conv_weight() -> np.ndarray:
    ci = 3      # number of input channels
    co = 12     # number of output channels
    step = torch.sqrt(torch.tensor(co / ci)).item()
    assert step % 1 == 0
    step = int(step)

    tiles = [torch.zeros(ci, ci, step, step) for _ in range(step**2)]
    for i, tile in enumerate(tiles):
        for j in range(ci):
            tile[j][j][i % step][i // step] = 1.0

    weight = torch.cat(tiles, 0).float()
    return weight.cpu().numpy()
```

这个过程涉及一个四维的张量，很难描述清楚，还得靠空间想象能力。这边尽力描述一些细节辅助读者理解，还望见谅。

1. `tiles` 的每个元素是一个 `ci x ci x step x step` 的张量，每次得到输出的3个通过，共 `step**2` 个元素。
2. `tile[j][j][i % step][i // step] = 1.0` 这个操作是将每个卷积核需要置1的位置置1.0，其他位置为0。
3. `[i % step][i // step]`：因为focus layer在cat的时候，顺序是先cat h 再cat w，所以这里h对应的位置是 `i % step`，w对应的位置是 `i // step`。
4. `[j][j]`：单个tile的输入通道和输出通道相等的index是1.0，其他位置是0。（这个比较难理解，需要自己画一个四维的张量）。

下面以yolov5为例子，画tiles的第一个元素tile张量，期望能帮助理解：

```plaintext
tile[0]:

[[[1.0, 0.0],
  [0.0, 0.0]],

 [[0.0, 0.0],
  [0.0, 0.0]],

 [[0.0, 0.0],
  [0.0, 0.0]]]

tile[1]:

[[[0.0, 0.0],
  [0.0, 0.0]],

 [[1.0, 0.0],
  [0.0, 0.0]],

 [[0.0, 0.0],
  [0.0, 0.0]]]

tile[2]:

[[[0.0, 0.0],
  [0.0, 0.0]],

 [[0.0, 0.0],
  [0.0, 0.0]],

 [[1.0, 0.0],
  [0.0, 0.0]]]
```

所以总是 `tile[j][j]` 是1.0，其他位置是0，这样就能实现focus layer的效果。

# 将归一化输入的操作融到卷积中

卷积的公式为：

$$
Y = W \cdot X + B
$$

由于归一化操作是对X进行线性变换，本身卷积也是线性变换，所以可以将归一化操作融到卷积中（两次线性变化可以一次做完），这样可以减少总体计算量，利于模型部署。

## 线性变化过程推导

一个卷积核的线性变化过程为，i代表行，j代表列，k代表通道：

$$
y = \sum_{k} \sum_{j} \sum_{i} w_{ijk} \cdot \frac{x_{ijk} - \mu_{k}}{\sigma_{k}} \\
= \sum_{k} \sum_{j} \sum_{i} \frac{w_{ijk}}{\sigma_{k}} \cdot x_{ijk} - \sum_{k} \sum_{j} \sum_{i} \frac{w_{ijk} \cdot \mu_{k}}{\sigma_{k}} \\
= \sum_{k} \sum_{j} \sum_{i} \frac{w_{ijk}}{\sigma_{k}} \cdot x_{ijk} - \sum_{k} \frac{\mu_{k}}{\sigma_{k}} \cdot W_{k} \\
 where \quad W_{k} = \sum_{j} \sum_{i} w_{ijk}
$$

因此 weight 和 bias 的修改过程为：

1. 每个通道的 weight 除以相应的 $\sigma_{k}$
2. bias 要减去 $\sum_{k} \frac{\mu_{k}}{\sigma_{k}} \cdot W_{k}$

## 代码实现

```python
def modify_weight_bias(weight: np.ndarray, bias: np.ndarray, mean: List[float], std: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    # weight: (co, ci, h, w), bias: (co,), mean: (ci,), std: (ci,)
    # 因为bias要用到weight，所以先处理bias
    for co in range(weight.shape[0]):
        for ci in range(weight.shape[1]):
            bias[co] -= weight[co, ci].sum() * (mean[ci] / std[ci])

    # 处理weight
    for ci in range(weight.shape[1]):
        weight[:, ci] /= std[ci]

    return weight, bias
```

# yolop使用样例

```python
import torch
import numpy as np
import onnx
from onnx import helper
from onnx import AttributeProto, TensorProto, GraphProto, NodeProto

from typing import *

def create_weight() -> np.ndarray:
    ci = 3
    co = 12
    step = torch.sqrt(torch.tensor(co / ci)).item()
    assert step % 1 == 0, "Matched part is not pixel unshuffle."
    step = int(step)

    tiles = [torch.zeros(ci, ci, step, step) for _ in range(step**2)]
    for i, tile in enumerate(tiles):
        for j in range(ci):
            # tile[j][j][i % step][i // step] = 1.0
            tile[j][ci - 1 - j][i % step][i // step] = 1.0

    weight = torch.cat(tiles, 0).float()
    return weight.cpu().numpy()

def modify_weight_bias(weight: np.ndarray, bias: np.ndarray, mean: List[float], std: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    # weight: (co, ci, h, w), bias: (co,), mean: (ci,), std: (ci,)
    # 因为bias要用到weight，所以先处理bias
    for co in range(weight.shape[0]):
        for ci in range(weight.shape[1]):
            bias[co] -= weight[co, ci].sum() * (mean[ci] / std[ci])

    # 处理weight
    for ci in range(weight.shape[1]):
        weight[:, ci] /= std[ci]

    return weight, bias


def create_conv_node(input_name: str, output_name: str, 
                     mean: List[float], std: List[float]) -> Tuple[NodeProto, TensorProto, TensorProto]:
    # 创建卷积节点的属性
    kernel_shape = [2, 2]  # 卷积核的形状
    strides = [2, 2]  # 步幅
    pads = [0, 0, 0, 0]  # 填充
    dilations = [1, 1]  # 膨胀率
    group = 1  # 分组卷积，默认为1
    auto_pad = 'NOTSET'  # 自动填充方式，默认为NOTSET

    # 从现有的权重张量创建初始化器节点
    weight_name = 'first_weight'
    weight_data = create_weight().astype(np.float32)
    bias_name = 'first_bias'
    bias_data = np.zeros((12,), dtype=np.float32)
    weight_data, bias_data = modify_weight_bias(weight_data, bias_data, mean, std)
    weight_initializer = onnx.numpy_helper.from_array(weight_data, name=weight_name)
    bias_initializer = onnx.numpy_helper.from_array(bias_data, name=bias_name)

    # 创建卷积节点
    conv_node = helper.make_node(
        'Conv',  # 节点类型
        inputs=[input_name, weight_name, bias_name],  # 输入的名称，需要根据实际情况修改
        outputs=[output_name],  # 输出的名称，需要根据实际情况修改
        name='conv',  # 节点名称
        kernel_shape=kernel_shape,  # 卷积核的形状
        strides=strides,  # 步幅
        pads=pads,  # 填充
        dilations=dilations,  # 膨胀率
        group=group,  # 分组卷积
        auto_pad=auto_pad  # 自动填充方式
    )

    return conv_node, weight_initializer, bias_initializer

def remove_slice_and_cat(graph:GraphProto, conv_o_name:str):
    remove_nodes = []
    for node in graph.node:
        if node.op_type == 'Slice':
            if node.input[0] == graph.input[0].name:
                remove_nodes.append(node)
        if node.op_type == 'Concat':
            if node.output[0] == conv_o_name:
                remove_nodes.append(node)

    for node in remove_nodes:
        graph.node.remove(node)

def slice2conv(graph: GraphProto, conv_o_name: str, 
               mean: List[float]=[0.485, 0.456, 0.406], std: List[float]=[0.229, 0.224, 0.225]):
    conv_i_name = graph.input[0].name
    conv_node, weight_initializer, bias_initializer = create_conv_node(conv_i_name, conv_o_name, mean, std)
    graph.node.insert(0, conv_node)
    graph.initializer.extend([weight_initializer, bias_initializer])

def main():
    onnx_model = onnx.load('yolop-640-640.onnx')
    conv_o_name = '/model.0/Concat_output_0'    # cat 算子的输出，现在要变成替换后的Conv算子的输出
    remove_slice_and_cat(onnx_model.graph, conv_o_name)

    # for input in [0, 1]
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    # [0, 1] -> [0, 255]
    mean *= 255
    std *= 255
    # input is bgr, not rgb
    mean = mean[::-1]
    std = std[::-1]
    slice2conv(onnx_model.graph, conv_o_name, mean, std)

    onnx.checker.check_model(onnx_model)
    onnx.save(onnx_model, 'yolop_modified.onnx')

if __name__ == "__main__":
    main()
```

以上的代码和之前细分拆解有两处区别，主要是因为yolop训练的时候输入是RGB，但是一般cv::imread 或者从 yuvToRGB 时得到的数据都是 BGR 格式的，多这一次转换就是多一次耗时，因此通过

1. `tile[j][j][i % step][i // step] = 1.0` 改成 `tile[j][ci - 1 - j][i % step][i // step] = 1.0` 就能在 focus 的时候顺带把通道顺序变了。
2. `mean = mean[::-1]` 和 `std = std[::-1]` 也需要做相应的修改，因为输入的通道顺序是BGR。

上面两处修改就避免了 BGR2RGB 的过程。

# 验证逻辑正确性

```python
def create_weight(input_rgb:bool=True) -> np.ndarray:
    ci = 3
    co = 12
    step = torch.sqrt(torch.tensor(co / ci)).item()
    assert step % 1 == 0, "Matched part is not pixel unshuffle."
    step = int(step)

    tiles = [torch.zeros(ci, ci, step, step) for _ in range(step**2)]
    for i, tile in enumerate(tiles):
        for j in range(ci):
            if input_rgb:
                tile[j][j][i % step][i // step] = 1.0
            else: # input is bgr
                tile[j][ci - 1 - j][i % step][i // step] = 1.0

    weight = torch.cat(tiles, 0).float()
    return weight.cpu().numpy()
def test():
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    # [0, 1] -> [0, 255]
    mean *= 255
    std *= 255
    input_rgb = False # True means input is RGB, False means input is BGR
    if not input_rgb:
        mean = mean[::-1]
        std = std[::-1]
    weight_name = 'first_weight'
    weight_data = create_weight(input_rgb).astype(np.float32)
    bias_name = 'first_bias'
    bias_data = np.zeros((12,), dtype=np.float32)
    weight_data, bias_data = modify_weight_bias(weight_data, bias_data, mean, std)
    weight_initializer = onnx.numpy_helper.from_array(weight_data, name=weight_name)
    bias_initializer = onnx.numpy_helper.from_array(bias_data, name=bias_name)
    input_tensor = onnx.helper.make_tensor_value_info('input', TensorProto.FLOAT, [1, 3, 640, 640])
    output_tensor = onnx.helper.make_tensor_value_info('output', TensorProto.FLOAT, [1, 12, 320, 320])
    conv_node = onnx.helper.make_node(
        'Conv',  # 节点类型
        inputs=['input', weight_name, bias_name],  # 输入的名称，需要根据实际情况修改
        outputs=['output'],  # 输出的名称，需要根据实际情况修改
        name='conv',  # 节点名称
        kernel_shape=[2, 2],  # 卷积核的形状
        strides=[2, 2],  # 步幅
        pads=[0, 0, 0, 0],  # 填充
        dilations=[1, 1],  # 膨胀率
        group=1,  # 分组卷积
        auto_pad='NOTSET'  # 自动填充方式
    )
    graph = onnx.helper.make_graph(nodes=[conv_node],  # 节点
                          name='conv',  # 图的名称
                          inputs=[input_tensor],  # 输入
                          outputs=[output_tensor],  # 输出
                          initializer=[weight_initializer, bias_initializer]  # 初始化器
                          )
    model =onnx.helper.make_model(graph, producer_name='onnx-example')
    onnx.checker.check_model(model)
    save_path = 'zz_fuse.onnx'
    onnx.save(model, save_path)

    input = np.random.randn(1, 3, 640, 640).astype(np.float32) * 255.0

    # model output
    ort_session = ort.InferenceSession(save_path)
    print(f"Load {save_path} done!")
    out = ort_session.run(['output'], {'input': input})
    print(out[0].shape)

    input_div = (input / 255.0)[:, ::-1, :, :].copy()
    input_div[:, 0] -= 0.485
    input_div[:, 1] -= 0.456
    input_div[:, 2] -= 0.406
    input_div[:, 0] /= 0.229
    input_div[:, 1] /= 0.224
    input_div[:, 2] /= 0.225
    input_div = torch.from_numpy(input_div).float()
    patch_top_left = input_div[..., ::2, ::2]
    patch_bot_left = input_div[..., 1::2, ::2]
    patch_top_right = input_div[..., ::2, 1::2]
    patch_bot_right = input_div[..., 1::2, 1::2]
    out_div = torch.cat([patch_top_left, patch_bot_left, patch_top_right, patch_bot_right], 1).cpu().numpy()
    print(out_div.shape)

    if np.allclose(out[0], out_div, atol=1e-5):
        print(r"""Conv node has substituted:
              1. divide input tensor by 255.0,
              2. substract mean and divide std,
              3. slice and concat.""")
    else:
        print("check failed.")
```

最终验证成功～
