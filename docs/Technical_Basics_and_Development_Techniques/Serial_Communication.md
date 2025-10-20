# 串口通信

在项目协作中，算法组一般担当数据处理的工作，然后将信息发给电控的单片机做控制，为了将信息发给电控，就需要使用到串口通信

与电控通信需要一个 `USB 转串口外设`，常用的有`CH340`和`USB线`，它的主要功能是在电脑的 USB 接口和设备的串口之间建立一个翻译通道，像电控使用的是串口语言，电脑使用的是 USB 协议，听不懂电控发来的消息，这时就需要一个翻译操作

- `串口`：全称串行接口，是一种逐位 发送和接收数据的计算机接口。顾名思义，数据像排队一样，一个接一个地按顺序传输
- `CH340`：是一个常用的串口转USB模块
- `USB线`：指 type-c / USB-A 转 micro USB / type-c 线，电控会使用一种名为虚拟串口的技术，将串口信息通过单片机上的 USB 口发送电脑能理解的 USB 协议

![ch340](images/Serial_Communication-image.png)

在和串口打交道中，有两个经常提及的参数——端口名称，波特率

- `端口名称（port_name）`：是插在电脑上串口的路径，在使用 CH340 时常出现在 `/dev/ttyUSB0` ，使用虚拟串口时则会是 `/dev/ttyACM0`
- `波特率（baud_rate）`：是两个设备见进行串口通信所协商的一个频率，波特率对上了才能收到通信消息

本教程主要学习如何对电控发来的信息进行解包和发包给电控，一个包有 8 比特（bit）/ 1 字节（byte）的容量大小，例如二进制$(1111 1111)_{2}=(255)_{10}$就有 8 位数（8 bit）

## 数据的存储

### int8_t

由于电控一次只能发 8 bit，所以我们在通信中规定一个包的数据类型为`int8_t`（有符号整型），与之相对的数据类型是`uint8_t`（无符号整型），符号指数值的正负，在一个包 8 比特的容量中，有 1 比特用来存储正负号，例如$(10000001)_{2}=(-1)_{10}$，`int8_t`和`uint8_t`的区别在于有一位拿来当正负号用了，所以`int8_t`只有 7 比特可以存储数字，`int8_t`的数字范围要在$-127——127$之间，$(0111 1111)_{2}=(+127)_{10}$

> 二进制的正负用 0 表示正数，1 表示负数

### int16_t

前面我们知道，int8_t 最大值就 127 ，但是我们传的角度范围在 $\pm 180$之间，明显数据位不够传了，这时有两种选择，一种使用 float32 ，一种使用 int16_t ,使用 float32 需要 4 个包，使用 int16_t 需要 2 个包。这里我们选择使用 int16_t ，一方面 float32 使用的包多，一方面对传输的角度数据 x100 再发送，由于 int16_t 范围 $\pm 32,768$，发送的角度数据最大 $18,000$ ，是不会发生溢出的

## 数据封装

### 数据读取

串口的开启代码详见附录，代码并不需要第三方库依赖。这里直接教怎么用附录封装好的类开启串口解包数据

```c++
#include "uart_transporter.hpp"

// 开启串口
string port_name = "/dev/ttyUSB0";  // 串口设备
int baud_rate = 115200;             // 波特率
auto uart_transporter = std::make_unique<UartTransporter>(port_name, baud_rate);

// 读取串口数据
#define capacity 16
uint8_t tmp_buffer_[capacity];
// 从串口读取 capacity 个包进 tmp_buffer_ 临时存储
int recv_len = uart_transporter->read(tmp_buffer_, capacity);
if (recv_len != capacity) return
```

这里的 uint8_t 也可以为 int8_t ，主要是为了从串口读取同等长度的二进制数进变量里

### 数据解包

我们知道，int16_t 需要 16 bit ，而电控发包一次只能 8 bit 地发，这时我们考虑将 16 bit 数据拆成两个 8 bit 数据两个包发送过来，然后我们收到两个包后将数据合并成一个

这里举个例子，电控发送一个 int16_t 的数据，$(1000\ 1111\ 1100\ 1100)_{2} = (-4044)_{10}$ ，我们将数据拆成两个包发送，即两个 int8_t ，`1000 1111`（高八位） 和 `1100 1100`（低八位），根据权重我们称呼拆开的两个包为高八位低八位，在发送时，可以先发高八位再发低八位，也可以先发低八位再发高八位

由于高八位、低八位的顺序不同，所以不同的解包方式会影响到数值的正确与否

为了将两个 int8_t 合并成 int16_t ，需要使用到**位运算符**，将高八位左移 8 位，再按位或第八位即可合并成功，例如 `1000 1111 << 8 = 1000 1111 0000 0000`，`1000 1111 0000 0000 | 1100 1100 = 1000 1111 1100 1100`

接下来我们完整地将电控发来的数据转换成 float32

```c++
int16_t yaw = (tmp_buffer_[1] << 8 ) | tmp_buffer_[2];
float yaw_receive = static_cast<float>(yaw); // 如果电控发来的数据有 x100 ，这里还需要 /100
```

## 数据发包

接下来我们讲如何将数据发送出去，我们常用的都是 int float double 数据类型，这些最少都需要 4 字节存储，但由于我们发送的数字范围能在 2 字节完成，所以这里不考虑数据溢出问题

下面演示如何将数据转成 int16_t ，再拆成 int8_t 包装

```c++
//初始化数据包
uint8_t tmp_buffer_[capacity]
for(int i = 0; i < capacity; i++) tmp_buffer_[i] = 0x00;

// 定义数据
float num = 233.233;
int16_t yaw = num*100;
//打包数据
tmp_buffer_[0] = yaw >> 8;     // 高八位
tmp_buffer_[1] = yaw & 0xFF;   // 低八位

uart_transporter->send(tmp_buffer_, capacity);
```

## 数据检测

我们有时需要发送一连串的包，然后接收解析，那么这有个问题，我们怎么知道这串包的第一位在哪里，这时便使用到了针头检测，在发送的第一个包写入独特的数据信息，以帮助确认开始解析的位置，一般在知道数据包长度后有针头检测就够了，但我们也可以在加入一步针尾检测

以下是例子

```c++
// 发送
tmp_buffer_[0] = 0xff;         // 帧头
tmp_buffer_[1] = yaw >> 8;     // 高八位
tmp_buffer_[2] = yaw & 0xFF;   // 低八位
int recv_len = uart_transporter->write(tmp_buffer_, capacity);

// 接收
uint8_t tmp_buffer_[capacity];
int recv_len = uart_transporter->read(tmp_buffer_, capacity);
if (tmp_buffer_[0] != 0xff) continue;
int16_t yaw = (tmp_buffer_[1] << 8 ) | tmp_buffer_[2];
float yaw_receive = static_cast<float>(yaw);

```

## 冗余校验

有了针头针尾检测后，我们可能还觉得不够，万一数据在传输过程中收到干扰，0 变成 1 这样怎么办，也就有了下面的对数据包进行检验的步骤

我们在发送的一串数据包中，第一个包做针头，倒数第一个包做针尾，倒数第二个包做数据校验

以下是一个简单的检验方式，异或校验

```c++
data[8] = data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];
```

除此之外的校验方式还有，奇偶校验，CRC 校验

## 附录

uart_transporter.hpp

```c++
// Copyright (C) 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_UART_TRANSPORTER_HPP_
#define SERIAL_DRIVER_UART_TRANSPORTER_HPP_

// std
#include <string>
// project


// 串口数据传输设备，符合通用传输接口。
class UartTransporter {
public:
    UartTransporter(
        const std::string& device_path = "/dev/ttyUSB0",
        int speed = 115200,
        int flow_ctrl = 0,
        int databits = 8,
        int stopbits = 1,
        int parity = 'N'
    ):
        device_path_(device_path),
        speed_(speed),
        flow_ctrl_(flow_ctrl),
        databits_(databits),
        stopbits_(stopbits),
        parity_(parity) {}

    bool open();
    void close();
    bool isOpen();
    int read(void* buffer, size_t len);
    int write(const void* buffer, size_t len);
    std::string errorMessage() {
        return error_message_;
    }

private:
    bool setParam(
        int speed = 115200,
        int flow_ctrl = 0,
        int databits = 0,
        int stopbits = 1,
        int parity = 'N'
    );

private:
    // 设备文件描述符
    int fd_ { -1 };
    // 设备状态
    bool is_open_ { false };
    std::string error_message_;
    // 设备参数
    std::string device_path_;
    int speed_;
    int flow_ctrl_;
    int databits_;
    int stopbits_;
    int parity_;
};


#endif // SERIAL_DRIVER_UART_TRANSPORTER_HPP_

```

uart_transporter.cpp

```c++
// Copyright (C) 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#include "uart_transporter.hpp"
// System
#include <errno.h> /*错误号定义*/
#include <fcntl.h> /*文件控制定义*/
#include <stdio.h> /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h> /*PPSIX 终端控制定义*/
#include <unistd.h> /*Unix 标准函数定义*/


bool UartTransporter::setParam(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
    // 设置串口数据帧格式
    int speed_arr[] = { B921600, B115200, B19200, B9600, B4800, B2400, B1200, B300 };
    int name_arr[] = { 921600, 115200, 19200, 9600, 4800, 2400, 1200, 300 };
    struct termios options;
    // tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
    // 该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    if (tcgetattr(fd_, &options) != 0) {
        error_message_ = "Setup Serial err";
        return false;
    }
    // 设置串口输入波特率和输出波特率
    for (size_t i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        if (speed == name_arr[i]) {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }
    // 修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    // 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    // 设置数据流控制
    switch (flow_ctrl) {
        case 0: // 不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1: // 使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2: // 使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    // 设置数据位
    // 屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            error_message_ = "Unsupported data size";
            return false;
    }
    // 设置校验位
    switch (parity) {
        case 'n':
        case 'N': // 无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O': // 设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E': // 设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': // 设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            error_message_ = "Unsupported parity";
            return false;
    }
    // 设置停止位
    switch (stopbits) {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            error_message_ = "Unsupported stop bits";
            return false;
    }

    // 修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 传输特殊字符，否则特殊字符0x0d,0x11,0x13会被屏蔽或映射。
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; // 读取一个字符等待1*(1/10)s
    options.c_cc[VMIN] = 1; // 读取字符的最少个数为1
    tcflush(fd_, TCIFLUSH);

    // 激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        error_message_ = "com set error";
        return false;
    }
    return true;
}

bool UartTransporter::open() {
    if (is_open_) {
        return true;
    }
    fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd_) {
        error_message_ = "can't open uart device: " + device_path_;
        return false;
    }
    // 恢复串口为阻塞状态
    if (fcntl(fd_, F_SETFL, 0) < 0) {
        error_message_ = "fcntl failed";
        return false;
    }
    // 测试是否为终端设备
    // 避免自启动无法读取数据
    // if (0 == isatty(STDIN_FILENO)) {
    //   error_message_ = "standard input is not a terminal device";
    //   return false;
    // }
    // 设置串口数据帧格式
    if (!setParam(speed_, flow_ctrl_, databits_, stopbits_, parity_)) {
        return false;
    }
    is_open_ = true;
    return true;
}

void UartTransporter::close() {
    if (!is_open_) {
        return;
    }
    ::close(fd_);
    fd_ = -1;
    is_open_ = false;
}

bool UartTransporter::isOpen() {
    return is_open_;
}

int UartTransporter::read(void* buffer, size_t len) {
    int ret = ::read(fd_, buffer, len);
    // tcflush(fd_, TCIFLUSH);
    return ret;
}

int UartTransporter::write(const void* buffer, size_t len) {
    int ret = ::write(fd_, buffer, len);
    return ret;
}


```
