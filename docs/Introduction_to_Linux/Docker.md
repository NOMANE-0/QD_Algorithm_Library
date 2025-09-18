# Docker

Docker 可以让开发者打包他们的应用以及依赖包到一个轻量级、可移植的容器中，然后发布到任何流行的 Linux 机器上，也可以实现虚拟化。

容器是完全使用沙箱机制，相互之间不会有任何接口（类似 iPhone 的 app）,更重要的是容器性能开销极低。

## 安装

docker的安装分`Docker Engine`和`Docker Desktop`两种，这里只讲`Docker Engine`

- 法一：鱼香ros（推荐）

输入以下命令，选择docker安装

```bash
wget http://fishros.com/install -O fishros && . fishros
```

![鱼香ros](images/image-20250722140550329.png)

安装后还需要将用户加入 docker 用户组才能用 docker 命令

```bash
sudo usermod -aG docker $USER
```

## 常用命令

```bash
# 拉取镜像
docker pull <镜像名:版本>

# 查看已下载的镜像
docker images
docker image ls 

# 查看已启动了的容器
docker container ls

# 启动容器
docker start <容器名>

# 关闭容器
docker stop <容器名>

# 重启容器
docker restart <容器名>

# 进入容器终端
docker exec -it <容器名> bash

# 进入容器的logs
docker attach <容器名>

# 容器开机自启动
docker container update --restart=always <容器名>

# 将镜像打包成压缩文件
docker save <镜像名:版本> > xxx.tar

# 通过压缩包加载镜像
docker load < xxx.tar

# 重命名镜像名， TAG 由 docker images 看到
docker tag <TAG> <新镜像名:新版本>
```

## 容器构建

基础的容器构建命令如下

```bash
docker run <镜像名:版本>
```

但我们一般都不会运行这么简单的命令

下面给出一条构建命令来讲解

```bash
docker run -it --name rv_runtime_ \
--privileged  -p 2222:22 -p 8765:8765 --restart always \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ~/rmvision2025:/ros_ws \
slirute/qidian:latest \
bash
```

- `-it`：允许用户与容器进行交互，类似于一个终端会话
- `--name rv_runtime_`：为容器指定名称 `rv_runtime_`，便于后续管理（如启动/停止）
- `--privileged`：赋予容器特权模式（相当于root），允许容器访问`/dev`的设备（摄像头、USB设备等）
- `-p 2222:22`：将主机的 2222 端口映射到容器的 22 端口，访问主机的222端口就是访问容器的22端口
- `--restart always`：容器退出时自动重启
- `-v ~/rmvision2025:/ros_ws`：将主机的`~/rmvision2025`挂载到容器中的`/ros_ws`目录，这样容器能访问到主机的文件
- `slirute/qidian:latest`：镜像:版本 ，指定使用的 Docker 镜像
- `bash`：容器启动后执行的命令，容器需要一个进程维持运行，如果进程结束，容器就会关闭

### 网络模式

容器的网络模式分为NAT模式和镜像模式，NAT是由主机给容器分发地址，镜像则是和主机共用端口和ip地址

## VScode开发

## 已有镜像

```bash
# 2024赛季使用镜像
docker pull shark277/qidian:1.0

docker run -it --volume="/dev:/dev" --ipc=host --device /dev/dri --privileged  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v /dev/shm:/dev/shm  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  -v /home/blackpoor/dockerCode:/home --name qidian qidianimage bash

# 2025赛季使用镜像
docker pull slirute/qidian:latest

docker run -it --name rv_devel_ \
--privileged -p 2222:22 -p 8765:8765 \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ~/rmvision2025:/ros_ws \
slirute/qidian:latest \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

```

## dockerfile

`dockerfile`是构建镜像的文件名称，通过在`dockerfile`中编写命令来自定义构建镜像



## VScode 开发

- 安装vscdoe扩展`Docker`、`Dev Containers`

![image-20250722203104541](images/image-20250722203104541.png)

![image-20250722203114654](images/image-20250722203114654.png)

- 使用扩展控制容器和附加进容器（在vscode中使用容器里的文件和终端）

![image-20250722203357355](images/image-20250722203357355.png)

![image-20250722203438418](images/image-20250722203438418.png)



## 常见问题

### 无法开机自启loaded failed failed LSB

```bash
systemctl list-units --type=service # 查看已启动服务
# 出现如下则本教程适用
docker.service
loaded failed failed  LSB: Create lightweight, portable, self-sufficient containers.
```

运行`wget -qO- ``https://get.docker.com/`` | sh`即可解决

```bash
systemctl list-unit-files | grep docker //查看是否自启动

systemctl enable docker.service //运行开机自启
```

