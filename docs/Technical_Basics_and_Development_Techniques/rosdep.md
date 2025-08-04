# ROS依赖补全

在克隆下一个 ROS 项目，这个项目还会依赖其他的 ROS 包，一个个找依赖是效率极低的做法。 ROS 给我们提供了一个命令`dosdep`，可以自动寻找包需要的依赖并下载。

初次使用需要初始化,不过使用的源是外国的，这里 鱼香ROS 提供了自己的国内源`wget http://fishros.com/install -O fishros && . fishros`
 
```bash
rosdep init

rosdep update

```

在项目根目录下输入以下命令

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- `--from-paths`：指定要检查依赖项的路径，这里检查 src 目录
- `--ignore-src`：忽略已存在于src目录中的包
- `-r`：递归处理所有子目录中的包
- `-y`：自动回答"yes"确认安装，无需手动交互
