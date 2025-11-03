# 自瞄部署

部署自瞄到车上，会对每辆车做一些定制改动

调车时使用的软件为 [foxglove](https://foxglove.dev/) ，该软件与自瞄运行在同一设备时负载较大，容易造成卡顿，建议在自己的电脑上跑 foxglove ，然后 ssh 远程连接 NUC

## 电控要求

在调车前需要保证车的性能达标，以免对后续造成干扰

- 电控串口发送频率要为 1k Hz ，使每 1ms 都有一个对应值
- 子弹散布在五米内应尽可能小，尽可集中在一个弹丸大小
- 云台 yaw pitch 的响应要快，这个可以后续看发给电控的 yaw pitch 有没有跟上，建议电控 PID 加前馈

## NUC 设置

每台拿到手的新小电脑都要修改以下配置，以尽可能发挥小电脑的性能

详情见[NUC 设置](/environment_configuration/NUC_setting.md)

目的

- 解锁大功率模式，提高 NUC 性能
- 关闭 Intel 小核
- 风扇转速拉最高，提高散热

## 系统环境

该设置仅需要刷系统后操作一次即可，后续不需要再执行，刷的系统统一用户名`qidian`密码`qd`

由于使用的是 docker 部署自瞄，所以对系统的依赖较小，但还是尽量选择 Ubuntu 系统，自瞄的测试是在 Ubuntu 22.04 上进行的

### brltty

使用的是 Ubuntu22 的话需要输入下面命令，不然没有 /dev/ttyUSB

```bash
# 输入后重启拔插设备即可
sudo apt remove brltty
```

### SSH连接

该步骤是为了让自己的电脑能通过 SSH 连接控制 NUC ，并给 NUC 供网

- 设置 NUC 的静态 ip 为 `192.168.137.x`，并配置自己电脑的 ip 为`192.168.137.1`，关于 ip 的设置详见[网络共享](/environment_configuration/Shared_network)
- 开启 NUC 的 ssh 服务端，详见[SSH](/Introduction_to_Linux/SSH)

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

> 由于是私有仓库，需要配置密钥才能访问，配置教程见[SSH](/Introduction_to_Linux/SSH)里 ssh密钥部分

### docker

#### [安装 docker](/Introduction_to_Linux/Docker)

#### 拉取镜像

```bash
# x86
docker pull slirute/qidian:latest
# arm
docker pull slirute/qidian:arm64v8 
```

如果在自己电脑有镜像了的话可以在**自己电脑**上用这个方法传到 NUC，省去下载时间

```bash
docker save slirute/qidian:latest | ssh qidian@192.168.137.x "docker load"
```

#### 构建容器

- 转发 8765 端口用于 foxglove 连接
- 附加`.ros`用于保存日志
- 附加`/dev`用于访问串口和摄像头
- 附加自瞄代码到容器的`/ros_ws`目录，这个是必须，容器会自动`source /ros_ws/install/setup.bash`为全局变量

以下是容器构建命令

```bash
# 用 foxglove 调试时
docker run -it --name rv_devel_ \
--privileged  --network host \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ~/rmvision2025:/ros_ws \
slirute/qidian:latest \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

```bash
# 自启动
docker run -it --name rv_runtime_ \
--privileged  --network host \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ~/rmvision2025:/ros_ws \
slirute/qidian:latest \
bash -c "source /ros_ws/src/rm_upstart/rm_watch_dog.sh"
```

## 编译与运行

任何的修改**只需要**改动`src/rm_bringup/config`下的各个 `yaml` 文件即可

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

- `--symlink-install`：采用软连接的方式编译，直接修改 src 下的 py 和 yaml 文件不用编译即可生效
- `--parallel-workers`： 限制编译线程数，没 32G 不要一次性编译，不然加 swap

## 调车

调车需要按顺序来，调试过程中需要在自己电脑上开 foxglove 并用网线连接 NUC，在 foxglove 中查看可视化调试

调车前请确认 NUC 连接相机和电控的串口了

### 标定相机内参

先标定相机内参，并将内参写入`camera_info.yaml`

### 验证标定数据

1. 开启自瞄识别装甲板，如果没有串口的话先把`launch_params.yaml`里`virtual_serial`设置为`true`
2. 在 foxglvoe 打开`图像`面板查看话题`/armor_detector/result_img/compressed`、`原始消息`面板查看话题`/armor_detector/armors`、`参数`面板修改参数曝光`/camera_driver.exposure_time`和增益`/camera_driver.gain`
3. 增益拉到最高，然后调节曝光，改变装甲板的距离和角度，查看识别到的装甲板位置（norm）对不对，建议与测量值误差在 3 cm 以内
4. 验证数据准确后将现在的曝光、增益写入`node_params/camera_driver_params.yaml`

**测距不准的原因**

- 内参标错
- 曝光没给对
- 装甲板角点位置不对，可以尝试调节`node_params/armor_detector_params.yaml`

![陈君语录1](images/deploy_auto_aim-image.png)

### 验证坐标变化

验证电控发来的角度数据

改变云台的 yaw pitch 值，使用 foxglove 的`三维`面板查看坐标轴的变换是否正确

要求电控发来的数据，yaw 向左、pitch 向下、roll 左倾为正

### 标定相机外参

建立相机与云台之间的联系，修改`launch_params.yaml`中的`dom2camera`的`xyz`和`rpy`

以 ROS 坐标系为准

xyz 为相机光心相对于云台的位置，可以由机械图纸或实际测量得到，单位米

rpy 为相机与云台坐标系轴线的角度差，一般只需要调节 pitch，单位弧度制

![标定pitch](images/deploy_auto_aim-image-2.png)

在装甲板高度不变的情况下，移动装甲板或车车，使用 foxglvoe 查看话题`/armor_solver/measurement.z`的高度有没有变化，理想情况是值不变

### 调参

总共有两个地方需要调，一个是串口 serial 节点，一个是自瞄 armor_solver 节点

#### serail

串口节点需要调节的是`serial_driver_params.yaml`中的`timestamp_offset`，这是手动时间补偿，单位秒。

1. 开自瞄识别静止装甲板，装甲板不能离开图像视野
2. 在 foxglove 中使用`图表`查看`/armor_solver/measurement.x`的值（坐标变换后装甲板在世界坐标系下的 x ）
3. 左右晃动云台，使云台姿态发生变换，观察图像里曲线的波动
4. foxgleve `参数`面板改变`/serial_driver.timestamp_offset`的值降低曲线的波动范围

![君佬调车回放](images/deploy_auto_aim-image-1.png)

#### armor_solver

计算用于扩展卡尔曼的观测噪声 R 的数值

修改`armor_solver_params.yaml`中`ekf`的 `r_x`  `r_y` `r_z` `r_yaw` （卡尔曼的观测误差），一般 `r_x = r_y`

所以一般计算出`r_x`和`r_z`的值就行，`r_yaw`在有三分法求 yaw 的情况下误差很小到不用改

计算方法

通过观察卡尔曼预测的数据`/armor_solver/target.position`来进行计算，上面四个值分别对应 x y z yaw

- 识别静止装甲板
- 在 foxglove 图表中调出相应的值，记下此时的值（稳态值）
- 左右晃动云台，但装甲板不能离开图像
- 从开始晃动头到结束，记录这个过程中曲线的极大值和极小值，然后根据以下公式计算出误差，写入 yaml 文件里面

$$误差 = ((极大值-极小值)\div4)^{2}\div稳态值$$

### 打弹测试

调好后还需要打弹测试来进行角度补偿和延迟计算

修改`armor_solver_params.yaml`

#### 角度补偿

由于机械装配误差，相机并不会完全平行于枪管，肉眼不可见地存在一定的偏移

修改`armor_solver_params.yaml`里的`angle_offset`

1. 开自瞄击打 2-4 m 正对枪管的静止装甲板，观察子弹落点与装甲板中心的误差
2. 根据落点小幅度修改`angle_offset`，手动给 yaw 和 pitch 矫正角度，使其打到正中心

#### 预测延迟

修改`armor_solver_params.yaml`里的延迟`predict2send_delay`

开启自瞄，发子弹打击匀速运动的目标。最好等到跟随稳定后发弹，因为开始跟随时都会有点跟不上

如果预测总体跟不上调大该值，预测超前则调小

#### 发弹延迟

电控从接收到视觉发来的开火指令到正真发射子弹间存在一个延迟，称之为开火延迟

修改`armor_solver_params.yaml`里的延迟`controller_delay`

暂时修改`solver.top1.max_orientation_angle`的值为 0（只瞄中心）， `solver.top1.max_out_error`为 0.2 （收紧打击时机宽度）

击打旋转装甲板，观察子弹是提前还是滞后打到，相应的减小增大该值

!> 根据上交博客所说，[每隔几个小时，打弹测试部分就需要重新校准](https://sjtu-robomaster-team.github.io/rm-cv-std-how-to-adjust-parameters/)
