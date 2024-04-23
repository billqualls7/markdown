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

rosbag2tum

```bash
evo_traj bag floam_00_result.bag /odom --save_as_tum
```

真值tum格式的数据

```
python3 evo/contrib/kitti_poses_and_timestamps_to_trajectory.py /home/wy/SemanticKITTI/data_odometry_poses/dataset/poses/00.txt /home/wy/SemanticKITTI/dataset/sequences/00/times.txt tum_00_gt.txt
```

真值为参考 绘画

```bash
evo_traj tum odom_final.txt --ref=tum_00_gt.txt -p -a -s
```

计算误差

```bash
evo_ape tum tum_00_gt.txt floam_00_odom.txt -r full -va --plot --plot_mode xyz
```

