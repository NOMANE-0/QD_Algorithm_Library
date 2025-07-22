# 没有 ttyUSB

明明插入 CH340 了但是`ls /dev/ttyUSB*`为没有输出文件

```terminal
# 输入
sudo dmesg | grep brltty
# 输出
[ 7033.078452] usb 1-13: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
```

出现这样的输出就是驱动占用的问题

```terminal
# 输入后重启拔插设备即可
sudo apt remove brltty
```

