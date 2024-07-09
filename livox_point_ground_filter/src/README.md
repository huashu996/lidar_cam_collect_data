# BEVpointcloud_process

ROS package for ground filter and cluster focus on the BEV pointcloud

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p BEVpointcloud_process/src
   cd BEVpointcloud_process/src
   git@github.com:huashu996/BEVpointcloud_process.git
   cd ..
   catkin_make
   ```
## 运行
 - 启动`ground_filter`
   ```Shell
   roslaunch plane_ground_filter plane_ground_filter.launch
   ```
 - 启动`cluster`
   ```Shell
   roslaunch points_cluster points_cluster.launch
   ```

