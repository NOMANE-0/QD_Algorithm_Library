# swap分区扩容

# 查看当前swap

运行以下命令查看当前的 Swap 配置

```terminal
# 查看内存和swap
free -h

# 查看swap详细信息
sudo swapon --show
```

输出示例

```terminal
radxa@rock-5b:~/$ free -h
               total        used        free      shared  buff/cache   available
Mem:           3.8Gi       508Mi       2.8Gi       101Mi       741Mi       3.3Gi
Swap:          8.0Gi          0B       8.0Gi

radxa@rock-5b:~$ sudo swapon --show
[sudo] password for radxa:
NAME      TYPE SIZE USED PRIO
/swapfile file   8G   0B   -2
```

# **删除旧的 Swap 文件**

```terminal
sudo swapoff -v /swapfile  # 停用 Swap
sudo rm -f /swapfile       # 删除旧的 Swap 文件
```

# **创建新的 Swap 文件**

- 法一：使用 `fallocate`

```terminal
sudo fallocate -l 4G /swapfile  # 创建 4GB Swap（可替换为 8G、16G 等）
sudo chmod 600 /swapfile       # 设置权限
sudo mkswap /swapfile          # 格式化为 Swap
sudo swapon /swapfile          # 启用 Swap
```

- 法二：**使用 `dd`**

```terminal
sudo dd if=/dev/zero of=/swapfile bs=1M count=4096  # 4GB (count=8192 是 8GB)
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

# 永久生效swap

编辑 `/etc/fstab`，确保系统启动时自动挂载 Swap

```terminal
sudo nano /etc/fstab
```

在末尾添加

```terminal
/swapfile none swap sw 0 0
```