# SLAM轨迹评估工具evo

## 下载

pip3 install evo --upgrade --no-binary evo

或者避免环境冲突，创造一个新的虚拟环境



## 使用

nav_msgs/Odometry

```bash
evo_traj bag kitti18.bag /odom_final  /vehicle -p
```

/home/wy/SemanticKITTI/data_odometry_poses/dataset/poses  真值路径

