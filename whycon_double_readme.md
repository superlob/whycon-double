## 在ROS中配置whycon_double的环境

 1. 在您的ubuntu系统中下载最新的**whycon_ros**软件包和其依赖项。
  [下载链接地址](https://github.com/jiriUlr/whycon-ros)
 
 2. 准备两个已经标定好的相机，下载**usb_cam**软件包，用roslaunch指令测试这两个相机是否能够正常运行。
 
               `roslaunch usb_cam usb_cam-test.launch`
  
 3. 创建一个工作空间，导入**double_camera_sub**软件包和上述两个软件包，用`catkin_make`指令编译。


## 使用说明

 1. 首先确保环境已配置好，并且将两个相机摆放到合适的位置（重叠区域尽可能大）。
 2. 在两个相机都已标定好的情况下，使用roslaunch指令同时启动双相机。
 
     `roslaunch double_camera_sub usb_two_cammera_ros.launch`
 3. 输入下方的指令，该节点开始进行自动标定，在标定的过程中让**环形图案**在两个相机的相机中运动，尽可能多地以不同角度出现。
 
       `rosrun double_camera_sub pos_sub_ros`
 4. 标定完成后，终端会打印出两相机之间的**外参**(R和t)，然后您可以通过下面的指令启动**rviz**可视化查看两相机在空间下的相对位置以及环形图案的位置。
 
     `roslaunch double_camera_sub doub_rviz.launch`
       

## 话题
### published

 - visualization_marker1 - 发布到rviz的全局相机下观测到的环形图案的三维坐标
 - visualization_marker2 - 发布到rviz的另一相机下观测到的环形图案转化到全局相机下的三维坐标

### subscribed

 - /c1/whycon_ros/markers -全局相机观测到的环形图案的三维坐标
 - /c2/whycon_ros/markers -另一相机观测到的环形图案的三维坐标


