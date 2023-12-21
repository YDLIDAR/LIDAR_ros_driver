###### *需先安装LIDAR_SDK

## 

## 进入工程目录，编译

```
catkin_make
source devel/setup.bash
```

## 使用

```
roslaunch lidar_ros_driver lidar.launch
或
roslaunch lidar_ros_driver lidar_view.launch
```

输出数据如下表:

| 话题           | 类型                     |
| ------------ | ---------------------- |
| /scan        | sensor_msgs/LaserScan  |
| /point_cloud | sensor_msgs/PointCloud |

launch中主要参数含义如下表:

| 参数名称        | 参数含义                              |
| ----------- | --------------------------------- |
| ip          | 雷达的ip地址                           |
| port        | 雷达的tcp通信端口                        |
| lidar_type  | 雷达的类型, 目前默认为0                     |
| angle_min   | 最小扫描角度(单位°), 目前支持最小到-150          |
| angle_max   | 最大扫描角度(单位°), 目前支持最大到150           |
| range_min   | 最小扫描距离(单位米), 目前支持最小到0.1米          |
| range_max   | 最大扫描距离(单位米), 目前支持最大到25米           |
| frequency   | 雷达扫描转速(单位Hz),目前支持10~40Hz,默认20Hz   |
| sample_rate | 雷达采样频率(单位K，即每秒雷达输出的点数),目前支持20~40K |
