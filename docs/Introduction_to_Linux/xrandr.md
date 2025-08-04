# xrandr

X11 窗口系统配置管理工具

## 查看显示器

```bash
# 查看当前输出的显示器
echo $DISPLAY
# 指定输出的显示器
export DISPLAY=:0
# 查看显示屏输出的端口名
xrandr
```

输入如下

![命令输出](images/xrandr-image.png)

- `$DISPLAY`是我们在终端中想操作的显示器指代，会影响到后面`xrandr`控制的屏幕
- `DP-1`是目前在使用的显示器，在使用的显示器下面都会显示多个可选的分辨率

## 修改分辨率

输入用`xrandr`获得的分辨率数据，以下命令是修改成分辨率`1920x1080`和帧率`60`

```bash
xrandr --output HDMI-1 --mode 1920x1080 --rate 60
```

## 创建新分辨率

创建新分辨率时请保证`DISPLAY`存在，即必须接上物理显示屏，并设置终端环境的`DISPLAY`为物理显示器的

- 使用 cvt 生成对应的 Modeline

```bash
cvt 1024 600
# 输出如下

# 1024x600 59.85 Hz (CVT) hsync: 37.35 kHz; pclk: 49.00 MHz
Modeline "1024x600_60.00"   49.00  1024 1072 1168 1312  600 603 613 624 -hsync +vsync
```

- 添加新模式

```bash
xrandr --newmode "1024x600_60.00"   49.00  1024 1072 1168 1312  600 603 613 624 -hsync +vsync
```

- 添加到显示器

```bash
xrandr --addmode DP-1 "1024x600_60.00"
```

- 应用新分辨率

```bash
xrandr --output DP-1 --mode "1024x600_60.00"
```

## 开机自启

设置完分辨率后我们发现重启就失效了，需要把命令全部冲打一遍，如果想要开机就自动设置分辨率的话，可以参考以下设置方法

- 创建切换分辨率脚本

```bash
nano ~/set-resolution.sh
```

输入以下内容

```bash
#!/bin/bash
#sleep 5    # 等待 5s 延迟，可能需要
xrandr --newmode "1024x600_60.00"   49.00  1024 1072 1168 1312  600 603 613 624 -hsync +vsync
xrandr --addmode DP-1 "1024x600_60.00"
xrandr --output DP-1 --mode "1024x600_60.00"
echo "分辨率脚本已运行" >> ~/xrandr_log.txt
```

赋予可执行权限

```bash
chmod +x ~/set-resolution.sh 
```

- 创建自启动桌面文件

```bash
mkdir -p ~/.config/autostart
nano ~/.config/autostart/set-resolution.desktop
```

输入以下内容

```bash
[Desktop Entry]
Type=Application
Exec=/home/用户名/set-resolution.sh
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
Name=SetResolution
Comment=Set custom resolution at startup
```
