# Linux报错记录

存放在 Linux 使用中遇到的问题及其解决方法

## 没有 ttyUSB

明明插入 CH340 了但是`ls /dev/ttyUSB*`为没有输出文件

```bash
# 输入
sudo dmesg | grep brltty
# 输出
[ 7033.078452] usb 1-13: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
```

出现这样的输出就是驱动占用的问题

```bash
# 输入后重启拔插设备即可
sudo apt remove brltty
```

## 挂载硬盘错误error mounting

```bash
# 查看硬盘位置 例/dev/nvme0n1p1
sudo fdisk -l

# 安装修复工具
sudo apt install ntfs-3g 

# /修复硬盘，之后就可以正常挂载了
sudo ntfsfix /dev/nvme0n1p1 
```

## libxxx.so: cannot open shared object file: No file or directory

[【超详细教程】解决libxxx.so: cannot open shared object file: No file or directory-CSDN博客](<https://blog.csdn.net/m0_37605642/article/details/130068402?ops_request_misc=&request_id=&biz_id=102&utm_term=./libcasadi_conic_osqp.so>: can&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-130068402.142^v99^pc_search_result_base3&spm=1018.2226.3001.4187)
