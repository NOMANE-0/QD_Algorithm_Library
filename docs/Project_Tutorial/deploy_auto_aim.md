# 自瞄部署教程

本教程仅讨论在准备好的情况下的部署，部署过程中的问题参见测试

关于自瞄的启动和自启动参见自瞄仓库的`README.md`

## 相机内参

- 标定自瞄使用的相机，教程参见[相机标定](camera_calibration.md)

- 将标定的内参写入`src/rm_bringup/config/camera_info.yaml`中

## 相机外参-平移向量

- 使用[相机外参标定](Camera_extrinsic_calibration.md)中的`方案一`获得平移参数。使用`CS016-10UC`和`HF-E系列镜头`的话光心大约在光圈处（测试过该系列的 6mm、8mm、12mm）

- 将平移参数写入`src/rm_bringup/config/launch_params.yaml`里的`xyz`

## 相机外参-rpy

### pitch

- 修改`src/rm_bringup/config/launch_params.yaml`里`enable_pitch_calibration`为`true`

- 启动自瞄和云台跟随

- 云台跟随相对前后平移的装甲板（车或者靶车前后动）

- 将时间到后终端会输出的`Optimal gimbal2camera pitch`的值写入`src/rm_bringup/config/launch_params.yaml`里`rpy`里的`p`

- `enable_pitch_calibration`改回`false`

### yaw

- 修改`src/rm_bringup/config/node_params/armor_solver_params.yaml`里的`solver.bullet_speed`为实际弹速

- 开自瞄，启动云台跟随静止装甲板

- 发射子弹

- 根据子弹落点偏左偏右调节`src/rm_bringup/config/launch_params.yaml`里的`rpy`的`y`直到打到水平中心（调节一次重启自瞄应用更改）

### offset

- 继续开自瞄跟随打弹看落点是偏高偏低

- 调节`src/rm_bringup/config/node_params/armor_solver_params.yaml`里`solver.angle_offset`的` pitch轴补偿值`直到打到垂直中心

## 超调量

- 使用 Foxglove 或 PlotJuggler 查看`/armor_solver/cmd_gimbal.yaw_diff`的曲线

- 开自瞄跟随平移或旋转靶车，观察`yaw_diff`的大小

- 调节`src/rm_bringup/config/node_params/armor_solver_params.yaml`里的`solver.additional_prediction_time`使`yaw_diff`趋近零（一般调节 10-40 ms）

## 延迟

> 这部分根据需要调整

调节`src/rm_bringup/config/node_params/armor_solver_params.yaml`里的`solver.predict2send_delay`和`solver.controller_delay`

设本来要预测 `dt_pre` 后的装甲板位置

### predict2send_delay

实际预测 `dt_pre + predict2send_delay` 时的装甲板位置

### controller_delay

如果云台指向 `dt_pre + predict2send_delay` 时的装甲板范围内时，此时指向的也是`dt_pre + predict2send_delay + controller_delay` 后的装甲板范围内，则允许开火

---

好了，你已经部署调好自瞄了