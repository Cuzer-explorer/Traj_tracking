# 轨迹跟踪
* 该工作空间下包含teb_local_planner, rrt_dwa, PID_controller三个功能包
* teb_local_planner接收给定的目标点位姿，利用teb算法规划出小车轨迹
* rrt_dwa接收给定的目标点位姿，通过rrt算法规划出全局路径，dwa算法规划出局部轨迹
* PID_controller以teb/dwa轨迹中的速度（线速度、角速度）为目标速度，以小车当前速度为反馈，PID得到速度控制量，发给底盘


## 轨迹规划
## TEB算法
* TEB算法，源码 https://github.com/rst-tu-dortmund/teb_local_planner
### 使用

* 若第一次使用，运行以下命令安装依赖
```
rosdep install teb_local_planner
```

* 参数位于 cfg/TebLocalPlannerReconfigure.cfg，包括速度加速度约束、车体类型、轨迹、障碍物各方面参数
* CMakeLists.txt中添加了绝对路径 include_directories(/usr/include/pcl-1.8)，需根据自己pcl库版本修改

### 运行
```
roslaunch teb_local_planner test_optim_node.launch
```

## RRT-DWA算法
* 参数位于 param.yaml
### 运行
```
roslaunch rrt_dwa plan.launch
```

## PID控制
* 参数位于 controller.yaml
### 运行
```
roslaunch PID_controller controller.launch
```
