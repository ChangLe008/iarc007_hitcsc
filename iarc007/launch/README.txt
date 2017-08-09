程序运行须知：
roslaunch iarc007 iarc007_01.launch
rosrun iarc007 jrh
以上2条命令可以启动全部程序！

分布运行程序：
roslaunch dji_sdk sdk_manifold.launch
rosrun guidance guidance_pub
rosrun iarc007 hy_opencv_pub
rosrun iarc007 mjh_vision_pos
rosrun iarc007 hy_opencv_sub
rosrun guidance guidanceNodeTest
roslaunch realsense_camera realsense_r200_nodelet_standalone_manual.launch
rosrun iarc007 jrh

开机自启动运行程序：
在桌面菜单栏的 “启动应用程序” 软件中 勾选 ：

/home/hitcsc/catkin_ws/src/iarc007/run_iarc007 iarc007自启动程序文件

自启动程序关闭采用命令： killall -9 rosmaster
