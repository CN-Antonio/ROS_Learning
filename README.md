ROS_Learning
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
roscore
# 启动节点
rosrun {packagename} {packagename_node}
```
## 查看
```bash
rostopic list
rosnode list
```