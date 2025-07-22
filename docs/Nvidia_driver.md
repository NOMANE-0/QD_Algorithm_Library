# NVIDIA 驱动安装

如果看这教程之前装显卡驱动装不成功的话 例：无法安装nvidia-driver-550 依赖于 nvidia-dkms-550 (＜= 550.54.15-1)

## 解决方案

```Plain
sudo apt-get --purge remove "*nvidia*" "libxnvctrl*"
```

## 安装失败原因

gcc版本问题，虽然官方说6.0～13.0版本都支持。。。

但Ubuntu22.04自带的这个gcc-11.4就是不行。于是更新gcc-12成功再去安装，成功解决。

## 第一步：安装gcc12

```Shell
# 查看之前的版本
gcc --version
//gcc-11 (Ubuntu 11.4.0-1ubuntu1~22.04) 11.4.0
//Copyright (C) 2021 Free Software Foundation, Inc.

sudo apt-get install gcc-12 # 安装gcc12
# 将他添加到gcc备选组中，且设置优先级为100
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100
```

- `/usr/bin/gcc` 主链接
- `gcc` 组名
- `/usr/bin/gcc-12` 要添加进组的链接
- `100` 优先级，越高约优先

### 查看gcc的链接设置

```Shell
sudo update-alternatives --config gcc
```

## 第二步：重新安装NVIDIA-DRIVER

去n卡官网安装，不要通过apt安装

下载驱动的地址： https://www.nvidia.cn/download/index.aspx

安装方式

```Shell
# 添加执行权限
sudo chmod +x "NVIDIA-Linux-x86_64-550.78.run"
# 执行安装
sudo bash "NVIDIA-Linux-x86_64-550.78.run"
```