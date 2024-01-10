# 结合深度学习的语义激光SLAM系统研究

## Segmatic KITTI

### 标签

标签可视化 [Segmatic KITTI数据集简单使用-CSDN博客](https://blog.csdn.net/yue__ye/article/details/108874928)

标签内自带移动物体的标签

### 网络

RangeNet++ [PRBonn/lidar-bonnetal: Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving (github.com)](https://github.com/PRBonn/lidar-bonnetal)

将三维点云投影到二维图像上

LMNet [PRBonn/LiDAR-MOS: (LMNet) Moving Object Segmentation in 3D LiDAR Data: A Learning-based Approach Exploiting Sequential Data (RAL/IROS 2021) (github.com)](https://github.com/PRBonn/LiDAR-MOS)

利用激光雷达的序列信息生成残差图像分割动态物体与静态物体

## SLAM程序

### ssc_loam

使用了数据集的真值标签来验证语义信息在系统中的作用

没有开源网络结构部分的代码，根据论文应该是基于LMNet的改进，引入了MHEF模块

为了解决数据不平衡的问题，计算时对损失进行加权，以使比例较小的类具有更大的 loss，从而提高神经网络分割动态对象的能力。同时，它可以缓解数据不平衡的问题。

标签重映射移动的行人和载具映射到动态类，其余的类别映射到静态类。

![image-20231114215528280](F:\NEU\一种基于深度学习的激光 SLAM 导航框架\photo\image-20231114215528280.png)

意思是两个模型参数？

还是在原来的基础上训练？



投影程序

预处理程序

模型推理程序