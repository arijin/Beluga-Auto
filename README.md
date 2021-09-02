# Beluga-Auto
这是一个和Auto-Beluga感知系统的代码仓库。

### 编译方式

```bash
$catkin_make -DCATKIN_WHITELIST_PACKAGES="me120_msg" #先编译消息
$catkin_make -DCATKIN_WHITELIST_PACKAGES="" #再编译整个文件夹
```

### 运行方式

##### Ros 启动
```bash
$roscore
```

##### Play
```bash
$cd ~/dataset/mydata
$rosparam set /use_sim_time true
$rosbag play 2021-07-06-15-39-20.bag -l --pause --clock
```

##### Beluga-Auto
```bash
$cd ~/Coding/catkin_auto
$source devel/setup.bash
$rosrun me120_tf me120_tf_node
```

```bash
$cd ~/Coding/catkin_auto
$source devel/setup.bash
# $roslaunch segmenters_lib demo.launch
$roslaunch tracking_lib demo.launch
```

##### Beluga-Viz
```bash
$cd ~/Coding/catkin_ws
$source devel/setup.bash
$roslaunch me120_tool demo.launch
```

##### Beluga-Paint 区域选取
```bash
$cd ~/Coding/catkin_ws
$source devel/setup.bash
$rosrun interact range
```



