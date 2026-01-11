# nmcli WIFI管理工具

在 Linux 系统中，最常用且最强大的 WiFi 管理工具是 NetworkManager 提供的命令行界面 nmcli。此外，对于喜欢图形化界面但又必须在终端操作的用户，nmtui 也是一个很好的选择。

## 基本查看操作

* **查看 WiFi 网卡状态：**
```bash
nmcli device
```


* **扫描附近的 WiFi 热点：**
```bash
nmcli device wifi list
```



## 连接与断开

* **连接指定的 WiFi：**
```bash
# 将 <SSID> 替换为 WiFi 名称，<PASSWORD> 替换为密码
sudo nmcli device wifi connect "SSID_NAME" password "PASSWORD"
```


* **断开指定的网卡连接：**
```bash
# 假设你的网卡名称是 wlan0
sudo nmcli device disconnect wlan0
```


* **重新连接（开启/关闭 WiFi 模块）：**
```bash
nmcli radio wifi off  # 关闭 WiFi
nmcli radio wifi on   # 开启 WiFi
```



## 管理已保存的连接

* **查看所有已保存的连接：**
```bash
nmcli connection show
```


* **启用/禁用某个已保存的连接：**
```bash
nmcli connection up id "WiFi_Name"
nmcli connection down id "WiFi_Name"
```


* **删除某个已保存的连接：**
```bash
nmcli connection delete "WiFi_Name"
```



## 查看当前连接详细信息

如果你想查看当前连接的信号强度、频率或 IP 地址，可以使用：

```bash
# 查看详细设备信息
nmcli device show wlan0

# 或者使用传统的工具查看信号（如果已安装）
watch -n 1 iwconfig
```


**小贴士：**

* 如果你的网卡名称不是 `wlan0`，请先通过 `ip link` 或 `ls /sys/class/net` 确认。
* 在执行连接操作时，如果 SSID 包含空格，记得使用引号将其括起来。

