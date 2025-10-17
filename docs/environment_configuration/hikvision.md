# [海康工业相机](hikvision.md)

【FlowUs 息流】海康摄像头使用

<https://flowus.cn/share/8fd6ac41-363e-4535-834e-a1f34670697e>

## MVS

MVS 是海康的机器视觉工业相机客户端，里面包含了相机测试软件和 SDK ，以下是下载链接

[机器视觉工业相机客户端MVS V4.6.0（Windows）](https://www.hikrobotics.com/cn2/source/support/software/MVS_Win_STD_4.6.0_250902.zip)

[机器视觉工业相机客户端MVS V4.6.0（Linux）](https://www.hikrobotics.com/cn2/source/support/software/MVS_Linux_STD_V4.6.0_250808.zip)

[机器视觉工业相机客户端MVS V4.5.1（Windows](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_4.5.1_250317.zip)

[机器视觉工业相机客户端MVS V3.0.1 (Linux)](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_V3.0.1_241128.zip)

```bash
# 打开MVS
cd /opt/MVS/bin
./MVS
```

## Runtime

Runtime 是运行海康相机的运行库，仅包含基础的驱动和二进制库文件

[机器视觉工业相机SDK V4.6.1版本Runtime组件包（Linux）](https://www.hikrobotics.com/cn2/source/support/software/MvCamCtrlSDK_STD_V4.6.1_250902.zip)

[机器视觉工业相机客户端MVS V3.0.1 (Linux)](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_V3.0.1_241128.zip)

## 部署环境

先将海康 SDK 移到自己的工作空间里

```bash
cd /your/workspace/
mkdir -p hikSDK/lib/
cp -r /opt/MVS/include/ hikSDK/
cp -r /opt/MVS/lib/64/* hikSDK/lib/
```

CMakeLists.txt

```cmake
project(hik_demo)
add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include)
target_link_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/)

target_link_libraries(${PROJECT_NAME} PRIVATE MvCameraControl)
```

引用头文件

```c++
#include "MvCameraControl.h"
```
