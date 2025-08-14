# OpenCV

OpenCV是一个C++的库。opnencv中有两个库，一个是[基础库`opencv`](https://github.com/opencv/opencv)，包含大部分常用的函数,还有一个是[扩展库`opencv_contrib`](https://github.com/opencv/opencv_contrib)，包含大部分常用的函数,还有一个是[扩展库`opencv_contrib`](https://github.com/opencv/opencv_contrib)，这个库主要是为了使用GPU（cuda）来加速opencv图像处理

## apt 安装

使用`apt`安装预编译好的opencv，版本为4.5.4，想要更新的话需要自己编译

```terminal
sudo apt install libopencv-dev
```

## 源码编译安装（多版本共存）

### apt 依赖安装

在编译前需安装一些包，以保证一些 GUI 可视化的正常使用

```bash
sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev  libtiff5-dev libopenexr-dev libtbb-dev libavcodec-dev libavformat-dev libswscale-dev libgtk-3-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev pkg-config
```

### 编译

获取源码,`-b`确认拉下来哪个版本

```bash
git clone -b 4.5.4 https://github.com/opencv/opencv.git
```

编译安装

```bash
cd opencv
mkdir build && build
# 构建
cmake .. \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local/opencv4.5.4 \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_opencv_python3=ON
# 编译
make -j4    # 4线程编译，看你电脑内存选择加
# 安装
make install
```

- `CMAKE_BUILD_TYPE`:构建类型为 ​​Release​​ / Debug
- `CMAKE_INSTALL_PREFIX`:make install 的安装路径
- `OPENCV_GENERATE_PKGCONFIG`:​生成 pkg-config 的配置文件，对不使用 CMake 进行构建的项目可能很有用
- `BUILD_opencv_python3`:构建 Python3 绑定

> 更多 cmake 可配置选项参见[配置文档](https://docs.opencv.org/4.12.0/db/d05/tutorial_config_reference.html)

## 环境变量

编译好后还要指定环境变量，这样运行程序时才能找到动态库。注意路径要对，可以把命令放入`~/.bashrc`中省的每次使用都要声明变量

```bash
export PKG_CONFIG_PATH=/usr/local/opencv4.5.4/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/opencv4.5.4/lib:$LD_LIBRARY_PATH
```

检验安装

```bash
pkg-config opencv4 --modversion
```

or

```bash
cd ~/opencv/samples/cpp/example_cmake # 拉取的源码里的代码
cmake .
```

## CMaklists.txt

```cmake
cmake_minimum_required(VERSION 3.0.0) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(OpenCV) 

set(OpenCV_DIR /usr/local/opencv4.5.4/lib/cmake/opencv4/)   # 如果你自定义了安装路径要加
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} ) 

add_executable( ${PROJECT_NAME} *.cpp ) 
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
```

## 交叉编译

在 x86 下编译出 arm 架构的 OpenCV

OpenCV 的一些功能需要依赖其他的 apt 包，典型的就是 GUI ，但是编译时又没法安装 arm 的 apt 包，所以这些功能会有缺失

```bash
# 安装编译工具
sudo apt install g++-aarch64-linux-gnu gcc-aarch64-linux-gnu
# 用这行 camke 即可，其他的与正常编译一样
cmake .. \
    -DCMAKE_INSTALL_PREFIX=../aarch64_install \
    -DWITH_CUDA=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DCMAKE_TOOLCHAIN_FILE=../platforms/linux/aarch64-gnu.toolchain.cmake \
    -DCMAKE_C_COMPILER=/usr/bin/aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++
```
