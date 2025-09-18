# 相机标定

相机标定开源选择上分为使用 ROS 标定和使用 matlab 标定。matlbal 标定不如 ROS 方便，这里不做讨论。当然标定的原理都是一样的，甚至你可以用 opencv 自己写一个标定程序来标定

## 依赖

```bash
sudo apt install ros-${ROS_DISTRO}-camera-calibration
sudo apt install ros-${ROS_DISTRO}-camera-info-manager
sudo apt install ros-${ROS_DISTRO}-launch-testing-ament-cmake
```

## 发布图像话题

标定程序需要订阅一个图像话题。拉取相机的驱动包或自己编写，修改`config`中的曝光和增益使画面能看清标定板，或者使用`rqt`或`foxglove`来调节曝光和增益

### 海康工业相机

```bash
git clone https://github.com/chenjunnn/ros2_hik_camera.git
cd ros2_hik_camera
colcon build --symlink-install
. install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```

### USB 相机

```bash
git clone https://gitee.com/slime0rimiru0/usb_cam.git
cd usb_cam
colcon build --symlink-install
. install/setup.bash
ros2 launch usb_cam usb_cam.launch.py 
```

## 确认图像话题名称

```bash
# 确认话题发布
ros2 topic list
# 看能否订阅话题
ros2 topic hz /image_raw
```

## 启动相机标定节点

```bash
ros2 run camera_calibration cameracalibrator --size 8x8 --square 0.035 image:=/image_raw camera:=/image_raw --no-service-check
```

`--size`：棋盘格尺寸，注意不是指格子数量，是黑白格相交的角点数量

`--square`：棋盘格格子大小，单位米

`image:=`：用来进行标定的图像话题

`camera`：图像节点 node

`--no-service-check`：禁止在启动时检查 set_camera_info 服务，使用 Ubuntu24 的时候这个服务会找不到，所以禁用了，实际不需要这个服务

[官方标定指南](https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_mono.html)

## 标定技巧

- 在图像上改变标定板距离、角度，直到四条都绿,多标不同距离不同角度的图 ~~除了 Size 其他都是能攒满的，不知道为什么 Size 就是不行~~

- 满后点击`CALIBRATE`开始计算标定数据，此时 GUI 无响应，等待即可。标定期间终端会输出标定图像部分信息，可以看到标了多少张图像，这个标定时间很长（笔者的 cpu AMD 7940h 在收集一百多张耗时几分钟，两百多耗时 1h ，三百张 2h 多）~~其实是Ubuntu22的问题，用24就可以快很多了~~。完成后终端会输出标定的相机参数

![image-20250722224147566](images/image-20250722224147566.png)

- 标定完成后 GUI 可以查看重投影误差，误差越小越好，一般在 0.5 左右。然后按`SAVE`，会生成`/tmp/calibration.tar.gz`，里面有标定的图片和相机参数，将里面的`ost.yaml`直接复制进`camera_info.yaml`

> 解压命令 `tar -zxvf calibration.tar.gz`

![image-20250722224830243](images/image-20250722224830243.png)

## [标定板下载网站](https://calib.io/pages/camera-calibration-pattern-generator)
