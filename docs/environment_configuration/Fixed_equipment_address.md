# 固定设备地址

在使用串口时，我们会发现有时它的文件位置不是固定的，可能从`/dev/ttyUSB0`变成`/dev/ttyUSB1`，为此我们根据设备的独有信息使用 udev 规则将设备绑定要一个固定的位置

## 设备信息查看

- 使用`lsusb`查看设备号

```bash
qd2025@qd2025:~/$ lsusb
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 001 Device 003: ID 1a86:7523 QinHeng Electronics CH340 serial converter
Bus 001 Device 004: ID 0573:1573 Zoran Co. Personal Media Division (Nogatech) USB Audio and HID
Bus 001 Device 005: ID 8087:0026 Intel Corp. AX201 Bluetooth
Bus 001 Device 009: ID 1a86:7523 QinHeng Electronics CH340 serial converter
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 002 Device 002: ID 2bdf:0001 Hikrobot MV-CS016-10UC
```

这里可以看到设备`CH340`的识别号为`1a86:7523`

- 使用`udevadm info --attribute-walk /sys/class/tty/ttyUSB1 | grep KERNELS`查看设备更多信息，这里的`ttyUSB1`替换成你设备的位置

```bash
qd2025@qd2025:~/$ udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep KERNELS
    KERNELS=="ttyUSB0"
    KERNELS=="1-2:1.0"
    KERNELS=="1-2"
    KERNELS=="usb1"
    KERNELS=="0000:00:14.0"
    KERNELS=="pci0000:00"
```

记录下这里的`KERNELS=="1-2"`（代表USB口位置），不要用`1-2:1.0`，测过了不会生效

## 固定设备

- 在`/etc/udev/rules.d/`下创建`usb.rules`文件，文件名可以自定义，后缀是`.rules`即可，输入以下内容

这行的意思是把`/dev/ttyUSB`前缀的设备，同时插在`KERNELS=="1-2"` 的USB 口上，设备识别号为`1a86:7523`连接到`/dev/rm_usb0`文件上

```bash
# 创建文件
sudo nano /etc/udev/rules.d/usb.rules
# 写入
KERNEL=="ttyUSB*", KERNELS=="1-2", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="rm_usb0"
```

`KERNELS=="1-2"`：这是固定 usb 口的位置

`KERNEL=="ttyUSB*"`：``ttyUSB*`意思是选`/dev`目录下所有前缀是`ttyUSB`的设备

`SYMLINK+="rm_usb0"`：rm_usb0就是给设备新的名字

`MODE:="0777"`：相当于`chmod 777`，给权限

- 使用`ls -l /dev/rm_usb0`可以查看`rm_usb0`连接到了哪个`ttyUSB`

---

同理，如果只有一个设备，但是名字会变的话，那么我们可以这么写

```bash
KERNEL=="ttyUSB*", MODE:="0777", SYMLINK+="rm_usb0"
KERNEL=="ttyACM*", MODE:="0777", SYMLINK+="rm_usb0"
```

USB 是 CH340 的串口，ACM 是虚拟串口，我们同一时间只会插入一个设备，但是我们不想每次换设备都改一次串口位置就可以这么写

## 重新加载udev规则

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger -v --action=add
```
