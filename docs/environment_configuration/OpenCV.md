# OpenCV

OpenCV是一个C++的库。opnencv中有两个库，一个是[基础库`opencv`](https://github.com/opencv/opencv)，包含大部分常用的函数,还有一个是[扩展库`opencv_contrib`](https://github.com/opencv/opencv_contrib)，包含大部分常用的函数,还有一个是[扩展库`opencv_contrib`](https://github.com/opencv/opencv_contrib)，这个库主要是为了使用GPU（cuda）来加速opencv图像处理

## 安装

### apt安装

使用`apt`安装预编译好的opencv，版本为4.5.4，想要更新的话需要自己编译

```terminal
sudo apt install libopencv-dev
```

### make编译

## CMaklists.txt

```terminal
cmake_minimum_required(VERSION 3.0.0) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(OpenCV) 

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} ) 

add_executable( ${PROJECT_NAME} *.cpp ) 
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
```
