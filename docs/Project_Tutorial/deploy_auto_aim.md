# 自瞄部署

部署自瞄到车上，会对每辆车做一些定制改动

调车时使用的软件为 foxglove ，该软件与自瞄运行在同一设备时负载较大，容易造成卡顿，建议在自己的电脑上跑 foxglove ，然后 ssh 远程连接 NUC

## 电控要求

在调车前需要保证车的性能达标，以免对后续造成干扰

- 电控串口发送频率要为 1k Hz ，使每 1ms 都有一个对应值
- 子弹散布在五米内应尽可能小，尽可集中在一个弹丸大小
- 云台 yaw pitch 的响应要快，这个可以后续看发给电控的 yaw pitch 有没有跟上，建议电控 PID 加前馈

## NUC 设置

每台拿到手的新小电脑都要修改以下配置，以尽可能发挥小电脑的性能

详情见[NUC 设置](/environment_configuration/NUC_setting.md)

## 系统环境

由于使用的是 docker 部署自瞄，所以对系统的依赖较小，尽量选择 Ubuntu 系统，自瞄的测试是在 Ubuntu 22.04 上进行的

### SSH连接（可选）

- 开启 NUC 的 ssh 服务端，详见[SSH](/Introduction_to_Linux/SSH)
- 固定 NUC 的网口 ip 为 `192.168.137.x` ， 网关 `192.168.137.1` ， 子网掩码 `255.255.255.0`， DNS `192.168.137.1`
- 设置自己的电脑与 NUC 处于同一网段下，最简单的是拿网线连接 NUC 和电脑，设置电脑的网口静态地址为 `192.168.137.1` ，子网掩码 `255.255.255.0` ， 网关 `192.168.137.1`

### [安装 docker](/Introduction_to_Linux/Docker)

### 固定串口

串口每次出现的路径可能不同，平常可能是`/dev/ttyUSB0`、`/dev/ttyACM0`这样，但有时拔插后会变成`/dev/ttyUSB1`、`/dev/ttyACM1`。所以我们要给串口一个固定路径

固定串口路径的教程详见[固定设备地址](/environment_configuration/Fixed_equipment_address.md)

### 拉取代码

拉取代码前确认你有配置 git 密钥了，不然用 scp 把自己电脑的代码拷贝过去

```bash
# 网络克隆下来
git clone git@gitee.com:ouzhigui/rmvision2025.git ~/rmvision2025
# scp 传文件
scp -r rmvision2025/ qidian@192.168.137.x:~/
```

> 由于是非公开仓库，需要配置密钥才能访问，配置教程见[SSH](/Introduction_to_Linux/SSH)里 ssh密钥部分

### docker 配置

#### 拉取镜像

```bash
docker pull slirute/qidian:latest
```

如果在自己电脑有镜像了的话可以用这个方法传，省去外网下载

```bash
docker save slirute/qidian:latest | ssh qidian@192.168.137.x "docker load"
```

#### 构建容器

- 构建的容器不适用镜像模式，不然局域网里有多台车会串台
- 直接转发 ssh 的 22 端口到主机的 2222 用于ssh连接（用户名 root 密码 password）和使用`ssh -Y`来进行容器内的 x11 转发（用于直接使用容器进行相机标定）
- 转发 8765 端口用于 foxglove 连接
- 附加`.ros`用于保存日志
- 附加`/dev`用于访问串口和摄像头
- 附加自瞄代码到容器的`/ros_ws`目录，这个是必须，容器会自动`source /ros_ws/install/setup.bash`为全局变量

以下是容器构建命令

```bash
# 用 foxglove 调试时
docker run -it --name rv_devel_ \
--privileged -p 2222:22 -p 8765:8765 \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ~/rmvision2025:/ros_ws \
slirute/qidian:latest \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

```bash
# 赛场用自启动
docker run -it --name rv_runtime_ \
--privileged  -p 2222:22 -p 8765:8765 --restart always \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ~/rmvision2025:/ros_ws \
slirute/qidian:latest \
bash -c "source /ros_ws/src/rm_upstart/rm_watch_dog.sh"
```

## 自瞄代码设置

任何的修改只需要改动`src/rm_bringup/config`下的各个 `yaml` 文件即可

### 编译与运行

```bash
# 进入容器
docker exec -it rv_devel_ bash
# 编译
cd /ros_ws
colcon build --symlink-install --parallel-workers 4 
# 运行
ros2 launch rm_bringup bringup.launch.py
```

- `--symlink-install`：采用软连接的方式编译，这样不用没修改 py 或 yaml 都要重新编译
- `--parallel-workers`： 限制编译线程数，没 32G 不要一次性编译，不然加 swap

### 基础设置

#### 标定相机位置

建立相机与云台之间的联系，修改`launch_params.yaml`中的`dom2camera`的`xyz`和`rpy`

##### xyz

相机光心相对于云台 pitch 轴的三维位置，使用的是 ROS 坐标系，单位米

##### rpy

由于机械装配误差，相机并不是完全平行于枪管，存在一定的角度偏移，这里可以为零，由后面的打弹测试手动补偿角度来解决

如果要标定，可以只标定 pitch，单位弧度制，该步骤需要先完成下面的**识别调节**

![标定pitch](images/deploy_auto_aim-image-2.png)

### 识别调节

提高识别装甲板的准确度，以获得精准的位置信息

先**标定相机**，将标定结果写入`camera_info.yaml`,[标定教程](/Project_Tutorial/camera_calibration)

该步骤主要调节相机的曝光（exposure_time）和增益（gain），结果保存在`camera_driver_params.yaml`中

- 用 foxglove 查看图像话题`/armor_detector/result_img/compressed`
- 查看装甲板的可视化角点，查看原始消息`/armor_detector/armors`看识别到的装甲板 PNP 解算的距离对不对，正常应与实际测量距离存在一个稳态误差，误差不能太大（5cm 以内）
- foxglove 打开参数面板，调节`/camera_driver.exposure_time`和`/camera_driver.gain`改变识别效果

> 如何调节曝光增益？
>
> 先确定曝光值，该值不宜超过 3000 μs ，多了陀螺转速高时识别的灯条会有拖影，影响识别效果
>
> 然后调节增益
>
> 如何判断调节合适？
>
> 调节后查看`/armor_detector/armors`的装甲板信息，看识别静止装甲板的数值波动和准确度
>
> 或者查看`/armor_solver/measurement.x`的值的波动和准确度

![陈君语录1](images/deploy_auto_aim-image.png)

### 调参

总共有两个地方需要调，一个是串口 serial 节点，一个是自瞄 armor_solver 节点

#### serail

图像的时间戳要和当前时间的 ypd 对齐，从数据采集到接收数据存在一个延迟，在 1000 Hz 下该延迟可以忽略不计，使用默认的 0

串口节点需要调节的是`serial_driver_params.yaml`中的`timestamp_offset`，这是手动时间补偿，单位秒。

- 识别静止装甲板，装甲板不能离开图像视野
- 在 foxglove 中查看`/armor_solver/measurement.x`的值（坐标变换后装甲板在 odom 坐标系下的 x ）的**图表**
- 左右晃动车的头，使云台姿态发生变换
- foxgleve 参数面板改变`/serial_driver.timestamp_offset`的值 ，观察`measurement.x`的波动，减小波动范围

![君佬调车回放](images/deploy_auto_aim-image-1.png)

#### armor_solver

计算用于扩展卡尔曼的观测噪声 R 的数值

修改`armor_solver_params.yaml`中`ekf`的 `r_x`  `r_y` `r_z` `r_yaw` （卡尔曼的观测误差），一般 `r_x = r_y`

计算出`r_x`和`r_z`的值就行，在有三分法求 yaw 的情况下误差很小了，`r_yaw`可以不用改

计算方法

通过观察卡尔曼预测的数据`/armor_solver/target`来进行计算，上面四个值分别对应 x y z yaw

- 在 foxglove 图表中调出相应的值，记下此时的值（稳态值）
- 识别静止装甲板，左右晃动云台，但装甲板不能离开图像
- 从开始晃动头到结束，记录这个过程中曲线的极大值和极小值，然后根据以下公式计算出误差，写入 yaml 文件里面

$$误差 = ((极大值-极小值)\div4)^{2}\div稳态值$$

### 打弹测试

调好后还需要打弹测试来进行角度补偿和开火延迟计算

修改`armor_solver_params.yaml`里的角度补偿`angle_offset`和延迟`controller_delay`

#### 角度补偿

由于机械装配误差，相机并不会完全平行于枪管，肉眼不可见地存在一定的偏移

修改`armor_solver_params.yaml`里的`angle_offset`

通过开自瞄击打静止装甲板，观察子弹落点与装甲板中心的误差，修改`angle_offset`，手动给 yaw 和 pitch 矫正角度，使其打到正中心

#### 发弹延迟

电控从接收到视觉发来的开火指令到正真发射子弹间存在一个延迟，称之为开火延迟

修改`armor_solver_params.yaml`里的延迟`controller_delay`

foxglove 用参数面板调节`/armor_solver.solver.controller_delay`的值，单位秒

将 yaml 文件里的`solver.top1.max_orientation_angle`的值改成零，这样自瞄时打旋转装甲板枪管都会瞄中心

击打旋转装甲板，观察子弹是提前还是滞后打到，相应的减小增大该值

!> 根据上交博客所说，[每隔几个小时，延迟参数可能也需要重新测量](https://sjtu-robomaster-team.github.io/rm-cv-std-how-to-adjust-parameters/)
