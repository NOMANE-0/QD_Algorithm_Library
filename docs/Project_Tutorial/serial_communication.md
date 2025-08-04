# 串口通信

## 串口通信简介：

串行接口是一种可以将接收来自[CPU](https://baike.baidu.com/item/CPU/120556?fromModule=lemma_inlink)的并行数据字符转换为连续的串行数据流发送出去，同时可将接收的串行数据流转换为并行的数据字符供给CPU的器件。一般完成这种功能的电路，我们称为串行接口电路。

串口通信（Serial Communications）的概念非常简单，串口按位（bit）发送和接收字节的通信方式。

我们应用串口通信主要用于与嵌入式进行信息传输，嵌入式将算法需要的数据发送过来，算法在执行完程序后，将要嵌入式执行的命令发送过去，简单来讲是这样的，后续可以开发出更多的功能...

> [C++串口通信-CSDN博客](https://blog.csdn.net/wy749929317/article/details/123822353?ops_request_misc=%7B%22request%5Fid%22%3A%22171333713316800226592791%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=171333713316800226592791&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-1-123822353-null-null.142^v100^pc_search_result_base6&utm_term=串口通信C%2B%2B&spm=1018.2226.3001.4187)
> 
> [C++使用serial串口通信 + ROS2示例IMU串口驱动_c++串口通信-CSDN博客](https://blog.csdn.net/zardforever123/article/details/134227412?ops_request_misc=%7B%22request%5Fid%22%3A%22171333713316800226592791%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=171333713316800226592791&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-2-134227412-null-null.142^v100^pc_search_result_base6&utm_term=串口通信C%2B%2B&spm=1018.2226.3001.4187)

## 串口通信的应用：

我们使用串口通信是为了与电控进行信息传递，实现对机器人的控制，获取机器人的信息。

例如，视觉程序需要电控发送IMU数据，裁判系统数据，模式切换信息等，处理结束后，视觉程序给电控发送云台控制的姿态

## **Cutecom 串口工具安装与使用：**

> [[Ubuntu\]Cutecom工具安装与使用_cutecom怎么用-CSDN博客](https://blog.csdn.net/weixin_49553049/article/details/114877684?ops_request_misc=&request_id=&biz_id=102&utm_term=ubuntu安装cutecom&utm_medium=distribute.pc_search_result.none-task-blog-2allsobaiduwebdefault-1-114877684.nonecase&spm=1018.2226.3001.4187)

## **Linux 固定 USB 串口**

**Linux下重启或者USB重新插拔会造成USB设备节点ttyUSB名不固定的问题**

使用生成固定USB转串口设备节点解决该问题：

> [Ubuntu下串口号端口号ttyUSB0和ttyUSB1不固定?-CSDN博客](https://blog.csdn.net/m0_37827405/article/details/89287876?ops_request_misc=&request_id=&biz_id=102&utm_term=固定dev/ttyusb0串口发数据每次固定的对应的USB口&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-89287876.nonecase)