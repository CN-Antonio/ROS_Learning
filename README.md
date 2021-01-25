# ROS_Learning-Table of Contents
- [通信](#通信)
- [常用工具](#tools)
- [客户端库](#Client_Library)
- [坐标变换](#TF_坐标变换)
- [统一机器人描述格式](#URDF)
- [SLAM](#slam)

# Structure
package
    CmakeLists.txt(必需)(规定编译规则)  
    package.xml(必需)

# 命令
## 编译
```bash
# 编译
catkin_make
```
## 编辑
```bash
# 创建package
catkin_create_pkg {packagename} {depend1} {depend2}
# 删除package
```
## 运行
```bash
# 启动 ROS Master
roscore #顺带启动rosout(日志输出)、parameter server(参数服务器)
# 启动 Node节点
rosrun pkg_name(包名) node_name(节点名)
rosrun {packagename} {packagename_node}
# 批量启动(master+node*n)
roslaunch [pkg_name] [file_name.launch](启动规则)
```
## 查看
```bash
# Topic
rostopic list
# Node
rosnode list
```
# ROS Master
（节点管理器，控制中心）  
提供参数服务器，用于节点存储检索运行参数

# Node
执行单元，进程
## Publisher
## Subscriber

# 通信
## Topic（异步通信机制）
1. 过程：sub、pub注册节点，pub发布Topic，sub订阅topic，形成pub-sub通信
2. 发布/订阅模型，单向通信
### Message（话题数据）
- Topic内容的数据类型，topic的格式标准  
- 使用.msg文件定义  
- pkg/msg/*.msg  
1. bool, int8, int16, int32, int64,(uint)
2. float32, float64, string
3. time, duration, header
4. 可变长数组array[], 固定长度数组array[C]
- 例：（连续、高频）激光雷达、里程计发布数据
## Service（同步通信机制）
- （C/S模型）请求、应答式
- 使用.srv文件定义数据格式
- pkg/srv/*.srv
- 例：（偶尔调用）开关传感器、拍照、逆解运算
## Parameter Sever
- 使用YAML格式(字典，键值对)：key:'value'
- \<param>\<rosparam>标签于launch.xml
## Action
- 类似Service，带有状态反馈，可回传到client
- *.action文件，数据格式
- 例：导航

# Tools
- 仿真：Gazebo 
- 调试、可视化：Rviz、rqt  
  Rviz:左侧每个功能都是一个Subscriber  
  rqt_graph:显示通信架构  
    圆圈-节点 箭头-topic   
  rqt_plot:订阅topic，绘制曲线  
  rqt_console:查看日志
- 命令行：rostopic、rosbag……  
  rosbag:记录、回放数据流，*.bag文件
- 专用：Moveit！

# Client_Library
*类似API  
1. roscpp(C++)
- 执行效率高，适用图像处理、SLAM，代码稍繁
  - ros::init()
  - ros::master Namespace
  - ros:service Namespace
  - ros:names Namespace
1. rospy(python)
- 开发效率高，适用小工具，rosrun，rostopic

# TF_坐标变换
维护坐标变换的工具
## in C++
|名称|类型|
|---|---|
|向量|tf::vector3|
## in Python
向量、点、四元数、矩阵表示成数组形式  
Tuple、List、Numpy Array通用
# URDF

# SLAM
## 开源算法包
1. Mapping
   - Gmapping
   - Karto
   - Hector
   - Cartographer
2. Localization
   - AMCL 
3. Path Planning
   - Navigation
## Map
