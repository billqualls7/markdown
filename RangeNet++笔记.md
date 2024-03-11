# RangeNet++笔记

1. 点云转换为图像--球形投影
2. 二维语义分割
3. 投影回去3D
4. 后处理使用基于GPU的KNN消除不好的点和阴影



## 投影

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
\frac{1}{2}\left[1-\arctan (y, x) \pi^{-1}\right] w \\
{\left[1-\left(\arcsin \left(z r^{-1}\right)+\mathrm{f}_{\mathrm{up}}\right) \mathrm{f}^{-1}\right] h}
\end{array}\right)
$$

e代表反射强度

x y z r e生成一个张量（tensor） 

```math
<<<<<<< HEAD
[5, W, H]
``` 
 
=======
>>>>>>> a62ec6f5ad18ca5cc4301229b27f83463f122140

Velodyne HDL 64-E激光雷达具有64个激光器,将投影图像的宽度设置为64

同样，对于HDL 64-E，最大水平分辨率为0.35度。因此，在最坏的情况下，每个激光至少可以得到（360 / 0.35 = 1028）点





核心代码

```python

  def do_range_projection(self):
    """ Project a pointcloud into a spherical projection image.projection.
        Function takes no arguments because it can be also called externally
        if the value of the constructor was not set (in case you change your
        mind about wanting the projection)
    """
    # laser parameters
    fov_up = self.proj_fov_up / 180.0 * np.pi      # field of view up in rad
    fov_down = self.proj_fov_down / 180.0 * np.pi  # field of view down in rad
    fov = abs(fov_down) + abs(fov_up)  # get field of view total in rad

    # get depth of all points
    depth = np.linalg.norm(self.points, 2, axis=1)

    # get scan components
    scan_x = self.points[:, 0]
    scan_y = self.points[:, 1]
    scan_z = self.points[:, 2]

    # get angles of all points
    yaw = -np.arctan2(scan_y, scan_x)
    pitch = np.arcsin(scan_z / depth)

    # get projections in image coords
    proj_x = 0.5 * (yaw / np.pi + 1.0)          # in [0.0, 1.0]
    proj_y = 1.0 - (pitch + abs(fov_down)) / fov        # in [0.0, 1.0]

    # scale to image size using angular resolution
    proj_x *= self.proj_W                              # in [0.0, W]
    proj_y *= self.proj_H                              # in [0.0, H]

    # round and clamp for use as index
    proj_x = np.floor(proj_x)
    proj_x = np.minimum(self.proj_W - 1, proj_x)
    proj_x = np.maximum(0, proj_x).astype(np.int32)   # in [0,W-1]
    self.proj_x = np.copy(proj_x)  # store a copy in orig order

    proj_y = np.floor(proj_y)
    proj_y = np.minimum(self.proj_H - 1, proj_y)
    proj_y = np.maximum(0, proj_y).astype(np.int32)   # in [0,H-1]
    self.proj_y = np.copy(proj_y)  # stope a copy in original order

    # copy of depth in original order
    self.unproj_range = np.copy(depth)

    # order in decreasing depth
    indices = np.arange(depth.shape[0])
    order = np.argsort(depth)[::-1]
    depth = depth[order]
    indices = indices[order]
    points = self.points[order]
    remission = self.remissions[order]
    proj_y = proj_y[order]
    proj_x = proj_x[order]

    # assing to images
    self.proj_range[proj_y, proj_x] = depth
    self.proj_xyz[proj_y, proj_x] = points
    self.proj_remission[proj_y, proj_x] = remission
    self.proj_idx[proj_y, proj_x] = indices
    self.proj_mask = (self.proj_idx > 0).astype(np.int32)
```







## 二维语义分割

### Fully Convolutional Semantic Segmentation

网络在进行downsample和upsample时，只在宽度上进行操作

垂直方向只能采集64个值

加权损失函数

## 投影回去3D

### Point Cloud Reconstruction from Range Image



## 后处理

基于GPU的KNN搜索



```yaml
    用于推理，计算整个数据集的平均值和标准差
    img_means: #range,x,y,z,signal
      - 12.12
      - 10.88
      - 0.23
      - -1.04
      - 0.21
    img_stds: #range,x,y,z,signal
      - 12.32
      - 11.47
      - 6.91
      - 0.86
      - 0.16
```

