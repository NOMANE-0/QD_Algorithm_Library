# Fedora KDE 以太网网络共享下无法解析 `rock-5b.local` 的原因与解决记录

## 1. 问题现象

在 Fedora KDE 上，通过以太网给 Rock 5B 做网络共享后，可以直接通过 IP 访问 Rock 5B：

```bash
ping 192.168.137.167
ssh radxa@192.168.137.167
```

但是无法通过主机名访问：

```bash
ping rock-5b
ping rock-5b.local
avahi-resolve-host-name rock-5b.local
```

报错或超时：

```text
ping: rock-5b.local: 名称或服务未知
解析主机名 'rock-5b.local' 失败：已超时
```

Rock 5B 上确认主机名正常：

```bash
hostnamectl
```

输出中可以看到：

```text
Static hostname: rock-5b
```

并且 Rock 5B 的 Avahi 服务也在运行：

```bash
systemctl status avahi-daemon
```

## 2. 网络环境

Fedora 主机：

```text
Wi-Fi:    wlp4s0，连接外部网络
Ethernet: enp3s0，用于共享网络给 Rock 5B
```

Fedora 以太网共享地址：

```text
192.168.137.1/24
```

Rock 5B 获取到的地址：

```text
192.168.137.167/24
```

IP 连通性正常：

```bash
ping 192.168.137.167
ssh radxa@192.168.137.167
```

这说明问题不是网线、DHCP、SSH 或 IP 网络不通，而是 `.local` 主机名解析失败。

## 3. 根本原因

`rock-5b.local` 使用的是 mDNS，也就是 Avahi 提供的本地域名解析。

mDNS 使用 UDP 5353 端口，并通过局域网多播进行主机名发现。

Fedora KDE / NetworkManager 在启用“共享给其他计算机”后，会把共享用的以太网接口 `enp3s0` 放到 firewalld 的 `nm-shared` zone 中：

```bash
sudo firewall-cmd --get-active-zones
```

实际输出：

```text
FedoraWorkstation (default)
  interfaces: wlp4s0
nm-shared
  interfaces: enp3s0
```

之前虽然可能已经给默认 zone 加过 mDNS：

```bash
sudo firewall-cmd --add-service=mdns
```

但是这只作用于默认 zone，也就是 `FedoraWorkstation`，并没有作用到 `enp3s0` 所在的 `nm-shared` zone。

因此，Fedora 的共享网口 `enp3s0` 上的 mDNS 流量被防火墙限制，导致：

```bash
avahi-resolve-host-name rock-5b.local
```

一直超时。

## 4. 解决方法

给 `nm-shared` zone 放行 mDNS 服务：

```bash
sudo firewall-cmd --zone=nm-shared --add-service=mdns
sudo firewall-cmd --permanent --zone=nm-shared --add-service=mdns
sudo firewall-cmd --reload
```

然后重启 Fedora 上的 Avahi 服务：

```bash
sudo systemctl restart avahi-daemon
```

Rock 5B 上也重启 Avahi：

```bash
ssh radxa@192.168.137.167
sudo systemctl restart avahi-daemon
exit
```

## 5. 验证结果

执行：

```bash
avahi-resolve-host-name rock-5b.local
```

成功解析：

```text
rock-5b.local   192.168.137.167
```

然后测试 ping：

```bash
ping rock-5b.local
```

结果正常：

```text
PING rock-5b.local (192.168.137.167) 56(84) 字节的数据。
64 字节，来自 192.168.137.167: icmp_seq=1 ttl=64 时间=0.263 毫秒
64 字节，来自 192.168.137.167: icmp_seq=2 ttl=64 时间=0.353 毫秒
64 字节，来自 192.168.137.167: icmp_seq=3 ttl=64 时间=0.339 毫秒
64 字节，来自 192.168.137.167: icmp_seq=4 ttl=64 时间=0.268 毫秒
```

之后也可以直接使用主机名 SSH：

```bash
ssh radxa@rock-5b.local
```

## 6. 总结

本次问题的关键点是：

```text
IP 能通 ≠ 主机名解析一定能通
```

`192.168.137.167` 能 ping、能 SSH，说明 Fedora 和 Rock 5B 的普通 IP 网络是正常的。

`rock-5b.local` 不能解析，说明 mDNS/Avahi 出了问题。

真正原因是 Fedora 的以太网共享接口 `enp3s0` 被 NetworkManager 放到了 `nm-shared` firewalld zone，而 mDNS 服务没有在这个 zone 中放行。

最终修复命令是：

```bash
sudo firewall-cmd --zone=nm-shared --add-service=mdns
sudo firewall-cmd --permanent --zone=nm-shared --add-service=mdns
sudo firewall-cmd --reload
sudo systemctl restart avahi-daemon
```

修复后：

```bash
avahi-resolve-host-name rock-5b.local
ping rock-5b.local
ssh radxa@rock-5b.local
```

均可正常工作。
