## fusion

### 使用

1. 把fusion中model文件夹中的yolov5参数模型替换为NX下的engine模型。
2. 根据最新的相机内外参，修改`image_trans.cpp`57-67行的K_temp, D_temp和E_temp。(NX运行opencv读取yaml文件会报错，只能手动输入)
3. `catkin_make -DCATKIN_WHITELIST_PACKAGES="fusion"`编译fusion包
4. 确认`cloud_cluster_detect.launch`中的参数配置无误
5. 运行此节点`$roslaunch fusion cloud_cluster_detect.launch`

如果报错找不到``me120_msgs`相关文件则需先
`catkin_make -DCATKIN_WHITELIST_PACKAGES="me120_msgs"`；NX上的编译会比较慢，要耐心等待

### 测试

- 话题
  此fusion节点订阅`/rslidar_points`和`/usb_cam/image_raw`话题，发布`me120_msgs::img_objs`的`/objects`话题（bbox的种类、xywl、距离的信息）

- 共有两种测试方法：
  1、`rosbag play -l XXX.bag --pause`播放带有以上话题的bag，可以测试fusion节点；2、也可以打开激光雷达和相机节点，实时的跑fusion节点

- 可视化选择：
  `pcd_rviz.cpp`的131-183为可视化的代码，可以根据需求注释，包括**点云投影到图像**、**点云投影到bbox**、**识别+测距**三个可视化结果可以看。

