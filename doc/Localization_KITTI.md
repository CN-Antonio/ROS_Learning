# 一、
# 二、播放数据集

```bash
# 合并分卷
cat bag_file*>bag.tar.gz

## KITTI数据 转 ROS的bag文件
# 工具：kitti2bag，numpy版本>=1.12
sudo pip install -U numpy
sudo pip install kitti2bag
# RAW文件结构
2011_10_03
├── 2011_10_03_drive_0027_sync
|   ├── image_00
|   ├── image_01
|   ├── image_02
|   ├── image_03
|   ├── oxts
│   └── velodyne_points
├── calib_cam_to_cam.txt
├── calib_imu_to_velo.txt
└── calib_velo_to_cam.txt
# 开始转换：cd进入上一步目录对应的"2011_10_03"文件夹的上一级目录，输入下面的指令，就会自动开始转换
kitti2bag -t 2011_10_03 -r 0027 raw_synced
# 生成一个文件"kitti_2011_10_03_drive_0027_synced.bag"


## 测试 bag文件
# 启动ros
roscore
# 使用配置文件打开rviz
rviz -d display_bag.rviz
# 播放bag
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```

# 三、软件框架