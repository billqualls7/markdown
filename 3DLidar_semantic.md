# 3D LiDAR Data for Semantic Segmentation

## 1.综述

### 深度学习术语

**Pooling：**

降低特征图（Feature Map）的维度，在卷积神经网络中，池化操作通常紧跟在卷积操作之后，用于降低特征图的空间大小。

池化操作的基本思想是将特征图划分为若干个子区域（一般为矩形），并对每个子区域进行统计汇总

通过池化操作，可以： 1. 降低特征图的维度，减少网络中参数的数量，避免过拟合现象的发生， 2. 提高模型的计算速度和运行效率。

**上采样：**

将图像上采样到更高分辨率

**Unpooling：**

Pooling的逆运算，可称为反池化，但是池化是不可逆的，近似运算

**转置卷积（Transposed convolution）:**

用转置卷积进行上采样

一对多的映射关系

**skip connect：**

残差连接 其中F包括卷积、激活等操作
$$
y=F(x)+x
$$


**backbone：**

主干网络，提取特征的网络

**head：**

head是获取网络输出内容的网络，利用之前提取的特征，head利用这些特征，做出预测。

**neck:**

是放在backbone和head之间的，是为了更好的利用backbone提取的特征。



### 点云标注数据集 SemanticKITTI

标注 28 类语义，共 22 个 sequences，43000 scans

 **velodyne文件**是激光雷达的测量数据（绕其垂直轴（逆时针）连续旋转）

每一行代表

|  x   |  y   |  z   |  r   |
| :--: | :--: | :--: | :--: |

**calib文件**是相机、雷达、惯导等传感器的矫正数据





## 2. Segmentation in 3D LiDAR Data

### 算法流程

1. 点云转换为图像--球形投影

   1.1 激光雷达硬件特质

   Velodyne HDL 64-E激光雷达具有64个激光器,将投影图像的宽度设置为64

   同样，对于HDL 64-E，最大水平分辨率为0.35度。因此，在最坏的情况下，每个激光至少可以得到（360 / 0.35 = 1028）点，使用1024

   

2. 二维语义分割

   2.1 CNN网络

   SqueezeNet移植用于特征提取

   相关一系列网络基本只在水平方向进行下采样，高度远小于宽度

   使用fireModules 和fireDeconvs替换了卷积和反卷积层

3. 投影回去3D

4. 后处理使用基于GPU的KNN消除不好的点和阴影

### Sequence Information

- 距离图像提供空间信息

- 残差图像提供时间信息

  

### 距离图像 Range Image

距离图像的宽度和高度则用（h, w）表示。注意，由于图像的像素数量有限，而一次扫描得到的点的数量大于距离图像中像素的数量，所以存在多个点映射到同一个像素的情况。因此点云中的点将按照**深度降序排列**以保证映射到距离图像上的点都在传感器的视野当中。

r是点到坐标系原点的欧式距离，代表点的深度信息。

r是点的二范数，即

$$
r=\sqrt{x^2+y^2+z^2}
$$

$$
\left(\begin{array}{l}
u \\
v
\end{array}\right)=\left(\begin{array}{c}
\frac{1}{2}\left[1-\arctan (y, x) \pi^{-1}\right] *w \\
{\left[1-\left(\arcsin \left(z r^{-1}\right)+\mathrm{f}_{\mathrm{up}}\right) \mathrm{f}^{-1}\right] *h}
\end{array}\right)
$$

```python
# def range_projection(current_vertex, proj_H=64, proj_W=900, fov_up=3.0, fov_down=-25.0, max_range=50, min_range=2):
""" Project a pointcloud into a spherical projection, range image.
  Args:
    current_vertex: raw point clouds
  Returns:
    proj_vertex: each pixel contains the corresponding point (x, y, z, depth)
"""
# laser parameters已在配置文件中给出
current_vertex=np.random.randn(100,4)
proj_H=64
proj_W=900
fov_up=3.0
fov_down=-25.0
max_range=50
min_range=2

fov_up = fov_up / 180.0 * np.pi  # field of view up in radians
fov_down = fov_down / 180.0 * np.pi  # field of view down in radians
fov = abs(fov_down) + abs(fov_up)  # get field of view total in radians

# 求出各点的depth(r)
depth = np.linalg.norm(current_vertex[:, :3], 2, axis=1)  # 求r(sqrt(x^2+y^2+z^2))
current_vertex = current_vertex[(depth > min_range) & (depth < max_range)]  # 保留在规定范围内的点
depth = depth[(depth > min_range) & (depth < max_range)]   # 保留在规定范围内的r

# 得到雷达扫描的各点各分量信息
scan_x = current_vertex[:, 0]    # x坐标
scan_y = current_vertex[:, 1]    # y坐标
scan_z = current_vertex[:, 2]    # z坐标
intensity = current_vertex[:, 3] # 强度

# 计算每个点的角度
yaw = -np.arctan2(scan_y, scan_x)
pitch = np.arcsin(scan_z / depth)

# range image下的坐标
proj_x = 0.5 * (yaw / np.pi + 1.0)            # in [0.0, 1.0]
proj_y = 1.0 - (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]

# 横纵放缩到设定好的range image图像大小
proj_x *= proj_W  # in [0.0, W]
proj_y *= proj_H  # in [0.0, H]

# 将投影后的横纵坐标规整到[0,W-1]和[0,H-1]
proj_x, proj_y = np.floor(proj_x), np.floor(proj_y)
proj_x, proj_y = np.minimum(proj_W - 1, proj_x), np.minimum(proj_H - 1, proj_y)
proj_x = np.maximum(0, proj_x).astype(np.int32)  # [0,W-1]
proj_y = np.maximum(0, proj_y).astype(np.int32)  # [0,H-1]

# order in decreasing depth
order = np.argsort(depth)[::-1]   # 返回r降序排列的数在原数组中的索引
depth = depth[order]              # 将r降序排列
intensity = intensity[order]      # 以r降序为主键，将intensity重排序
proj_y = proj_y[order]            # 以r降序为主键，将proj_y重排序
proj_x = proj_x[order]            # 以r降序为主键，将proj_x重排序

scan_x = scan_x[order]            # 以r降序为主键，将scan_x重排序(点云坐标)
scan_y = scan_y[order]            # 以r降序为主键，将scan_y重排序(点云坐标)
scan_z = scan_z[order]            # 以r降序为主键，将scan_z重排序(点云坐标)

indices = np.arange(depth.shape[0])
indices = indices[order]

proj_range = np.full((proj_H, proj_W), -1,dtype=np.float32)      # [H,W] range (-1 is no data)
proj_vertex = np.full((proj_H, proj_W, 4), -1,dtype=np.float32)  # [H,W] index (-1 is no data)
proj_idx = np.full((proj_H, proj_W), -1,dtype=np.int32)          # [H,W] index (-1 is no data)
proj_intensity = np.full((proj_H, proj_W), -1,dtype=np.float32)  # [H,W] index (-1 is no data)

# range image中
proj_range[proj_y, proj_x] = depth
# proj_vertex中每个像素都保存着它在原始点云中的对应点的信息(x, y, z, depth)
proj_vertex[proj_y, proj_x] = np.array([scan_x, scan_y, scan_z, depth]).T  # (64, 900, 4)
proj_idx[proj_y, proj_x] = indices
proj_intensity[proj_y, proj_x] = intensity
```

### 残差图像 Residual Images

1. 手动生成残差图像

2. 帧之间的变换估计，统一坐标系，消除自我运动

3. 将过去帧投影到当前距离图像视图当中，计算当前帧与转换帧的距离（归一化、绝对差值）

   只计算有效像素的残差，并将无效像素的残差设置为0

$$
d_{k,i}^l=\frac{\left|r_i-r_i^{k\rightarrow l}\right|}{r_i}
$$

```python
# 对整个seq生成残差深度图像
  for frame_idx in tqdm(range(len(scan_paths))):
    file_name = os.path.join(residual_image_folder, str(frame_idx).zfill(6))
    diff_image = np.full((range_image_params['height'], range_image_params['width']), 0,
                             dtype=np.float32)  # [H,W] range (0 is no data)
    
    # 对于一开始的N个frame生成哑文件
    if frame_idx < num_last_n:
      np.save(file_name, diff_image)
    
    else:
      # 加载当前扫描并且调用range_projection()函数生成range image
      current_pose = poses[frame_idx]
      current_scan = load_vertex(scan_paths[frame_idx])
      current_range = range_projection(current_scan.astype(np.float32),
                                       range_image_params['height'], range_image_params['width'],
                                       range_image_params['fov_up'], range_image_params['fov_down'],
                                       range_image_params['max_range'], range_image_params['min_range'])[:, :, 3]
      
      # 加载上一个扫描帧, 转换到当前坐标系，然后生成相应的range image
      last_pose = poses[frame_idx - num_last_n]    
      last_scan = load_vertex(scan_paths[frame_idx - num_last_n])  # 前一个scan
      last_scan_transformed = np.linalg.inv(current_pose).dot(last_pose).dot(last_scan.T).T  # 转换到当前scan的坐标系
      # 调用range_projection()函数，生成range image
      last_range_transformed = range_projection(last_scan_transformed.astype(np.float32),
                                                range_image_params['height'], range_image_params['width'],
                                                range_image_params['fov_up'], range_image_params['fov_down'],
                                                range_image_params['max_range'], range_image_params['min_range'])[:, :, 3]
      
      # 生成二值化掩膜, 只保留图像中的有效位置部分:(min_range, max_range)
      valid_mask = (current_range > range_image_params['min_range']) & \
                   (current_range < range_image_params['max_range']) & \
                   (last_range_transformed > range_image_params['min_range']) & \
                   (last_range_transformed < range_image_params['max_range'])
      # 作差，得到残差图像
      difference = np.abs(current_range[valid_mask] - last_range_transformed[valid_mask])
      # 标准化，逐点除以当前图像的各点r值
      if normalize:
        difference = np.abs(current_range[valid_mask] - last_range_transformed[valid_mask]) / current_range[valid_mask]
      # 残差图像就是diff_image
      diff_image[valid_mask] = difference
      
      if debug:
        fig, axs = plt.subplots(3)
        axs[0].imshow(last_range_transformed)
        axs[1].imshow(current_range)
        axs[2].imshow(diff_image, vmin=0, vmax=10)
        plt.show()
        
      if visualize:
        fig = plt.figure(frameon=False, figsize=(16, 10))
        fig.set_size_inches(20.48, 0.64)
        ax = plt.Axes(fig, [0., 0., 1., 1.])
        ax.set_axis_off()
        fig.add_axes(ax)
        ax.imshow(diff_image, vmin=0, vmax=1)
        image_name = os.path.join(visualization_folder, str(frame_idx).zfill(6))
        plt.savefig(image_name)
        plt.close()

      # save residual image
      np.save(file_name, diff_image)
```



## 3. Lidar-MOS复现

### 标签

标签可视化 [Segmatic KITTI数据集简单使用-CSDN博客](https://blog.csdn.net/yue__ye/article/details/108874928)

标签内自带移动物体的标签

### 网络

RangeNet++ [PRBonn/lidar-bonnetal: Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving (github.com)](https://github.com/PRBonn/lidar-bonnetal)

将三维点云投影到二维图像上

LMNet [PRBonn/LiDAR-MOS: (LMNet) Moving Object Segmentation in 3D LiDAR Data: A Learning-based Approach Exploiting Sequential Data (RAL/IROS 2021) (github.com)](https://github.com/PRBonn/LiDAR-MOS)

利用激光雷达的序列信息生成残差图像分割动态物体与静态物体

#### 环境问题

系统配置

|      Ubuntu      |       16.04        |
| :--------------: | :----------------: |
| GeForce RTX 3090 | CUDA Version: 11.1 |
|     PyTorch      |    1.10.1+cu111    |
|      scipy       |       1.2.0        |
|       onnx       |       1.8.0        |

使用conda移植环境ssc

```bash
ImportError: No module named 'scipy.spatial.transform'

pip3 install scipy==1.2.0
pip3 install onnx==1.11.0
```

在PRBonn开源项目中所有数据集路径均表示为-d /data_jiang/wy/dataset/SemanticKITTI/dataset

推理：./infer.py -d /data_jiang/wy/dataset/SemanticKITTI/dataset/ -l log_pre/ -m log/



- [x] 复现语义模型
- [ ] 复现静态动态分类模型
- [ ] 涉及标签映射、重映射、逆映射



