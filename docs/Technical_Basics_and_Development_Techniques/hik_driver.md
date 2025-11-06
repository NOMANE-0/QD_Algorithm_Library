# 海康工业相机驱动

> 本节将学习如何驱动海康工业相机，以及如何将数据转化成给 OpenCV 用的数据格式
>
> 学习前请确保配好环境，详细见[海康工业相机](/environment_configuration/hikvision)

海康相机的驱动主要分成枚举相机、打开相机、设置相机、开始取流、获得图像、释放图像、关闭相机这几步骤

海康的 MVS 里有许多案例可以参考学习，建议观看`/opt/MVS/Samples/README-CH`来索引，本节的代码大多来自`/opt/MVS/Samples/32/C++/General/ImageProcess/ImageProcess.cpp` 

## 头文件

```c++
#include "MvCameraControl.h"
```

## 获得句柄

句柄是对系统资源的抽象引用或标识符，是与海康工业相机进行通信和控制的唯一通行证

```c++
// 初始化SDK
int nRet = MV_CC_Initialize();

// 枚举设备
MV_CC_DEVICE_INFO_LIST stDeviceList;
int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);

// 选择设备并创建句柄
void* handle = NULL;
int nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);

// 不使用句柄了需要销毁掉
int nRet = MV_CC_DestroyHandle(handle);
```

每一个 SDK 函数都会返回一个错误码，表示该函数的运行状态，成功无异常为`nRet = MV_OK`

- `MV_CC_EnumDevices`：枚举设备获得当前所有连接的海康设备列表，这里只搜索 USB 相机，`stDeviceList`是一个设备信息列表数组
- `MV_CC_CreateHandle`：根据枚举的相机信息创建一个句柄

## 打开设备

获得句柄后需要打开相机，以让句柄获得操作相机的权限，开启成功后其他句柄无法开启该相机，最终有且仅有一个句柄能控制相机

```c++
// 开启设备
int nRet = MV_CC_OpenDevice(handle);
// 关闭设备
int nRet = MV_CC_CloseDevice(handle);
```

## 设置相机参数

开启相机后，我们还需要对相机的一些参数作个性化设置，像基本的曝光、增益

```c++
// 获得曝光属性值
MVCC_FLOATVALUE exposure_time_value;
int nRet = MV_CC_GetFloatValue(handle, "ExposureTime", exposure_time_value)
std::cout << "ExposureTime = " << exposure_time_value.fCurValue << std::endl;
// 设置曝光
double exposure_time = 1000;
int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time);
```

获得和设置的函数如上所示，不同的相机参数有不同的数据类型，相应的函数名和输出的数据类型也不同，具体需要看`MvCameraControl.h`里的定义

设置相机参数需要的参数名字和数据类型通过查阅`/opt/MVS/doc/Machine Vision Camera SDK Developer Guide Linux (C) V4.5.0/Machine Vision Camera SDK Developer Guide Linux (C) V4.5.0.html`里的 Camera Parameter Node Table 可以得到

## 取流获取图像数据

取流是从相机设备获取实时或压缩的视频数据流（Video Data Stream）的过程，即相机开始工作采集图像

```c++
// 开始取流
int nRet = MV_CC_StartGrabbing(handle);
// 获得图像缓存
MV_FRAME_OUT stImageInfo;
int nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
// 释放图像缓存
int nRet = MV_CC_FreeImageBuffer(handle, &stImageInfo);
// 停止取流
int nRet = MV_CC_StopGrabbing(handle);
```

取流和停止取流一般位于构造和析构函数，获得图像缓存后像再次获得图像缓存必须先释放图像缓存，1000 是等待图像的时间（ms）

## 图像格式转换

因为默认海康相机的图片格式是 
获得了图像缓存（stImageInfo）就可以作格式转换了，SDK 有提供现成的函数来转换,

```c++
// 像素格式转换
unsigned char *pDataForRGB = (unsigned char*)malloc(stImageInfo.stFrameInfo.nExtendWidth * stImageInfo.stFrameInfo.nExtendHeight * 4 + 2048);
MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
// 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
// 目标像素格式，输出数据缓存，提供的输出缓冲区大小
stConvertParam.nWidth = stImageInfo.stFrameInfo.nExtendWidth;
stConvertParam.nHeight = stImageInfo.stFrameInfo.nExtendHeight;
stConvertParam.pSrcData = stImageInfo.pBufAddr;
stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;
stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
stConvertParam.pDstBuffer = pDataForRGB;
stConvertParam.nDstBufferSize = stImageInfo.stFrameInfo.nExtendWidth * stImageInfo.stFrameInfo.nExtendHeight *  4 + 2048;
nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);

cv::Mat dst_image(
            cv::Size(stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight),
            CV_8UC3,
            pDataForRGB
            );
```
