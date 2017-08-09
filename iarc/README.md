# 本文档用于说明控制程序结构

## 整体结构

1. 所有程序文件都保存在`src/pilot`文件夹下
2. 主程序为 `main.cpp`
3. 控制部分主要在`control.cpp`下
4. 控制部分频率为50Hz
5. 现已注释自动起飞部分代码段为
    ``` c
    	if(drone->takeoff())
	{
		ROS_INFO("Takeoff command is sent");
	}
	else
	{
		ROS_INFO("Unable to take off");
	}
    ```
6. 所有订阅信息与信息处理部分都在`subscribe.cpp`文件夹下、
7. `control.cpp`中最关键变量为`control_mode`代表当前飞行棋状态，其有五个值，1,2,3,4,5，分别代表45度交会180度交会，跟踪，巡航和上升。在五个mode中分别调用相应的子函数实现功能，其中45度交会和180度交会分别有一个子状态变量`stage_45`和`stage_180`标记当前所处交会状态。
8. 障碍物处理程序放在`rplidar_ros`包`leo_pub.cpp`中，当前只能返回一个障碍物位置，以后改进是加入返回多个障碍物。


## 关于结构的一些建议 
1. 建议把所有信息处理部分放在`subscribe.cpp`里
2. 建议把障碍物信息处理部分放在`leo_pub.cpp`中
3. 建议把顶层策略写成50Hz放在 `control.cpp`中
```
/// Leo strategy recommended

部分。