# 网络共享

负责共享网络的设备称之为服务端，想要联网的设备称为客户端，服务端需要有有线网卡和无线网卡

## Linux

- 环境：Ubuntu 22.04

### 服务端

打开“网络连接”设置网络共享

```bash
nm-connection-editor
```

打开设置

![设置](images/Shared_network-image.png)

在`IPv4`设置中设置`方法`为`与其他计算机共享`，`添加` ip 地址和子网掩码

![ipv4](images/Shared_network-image-1.png)

### 客户端

和服务端操作一样打开网络设置，并设置静态 ip

![ip](images/Shared_network-image-2.png)