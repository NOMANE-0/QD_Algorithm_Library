# 开机自启动

在写完程序后，我们希望它能开机自动运行，以下提供几种开机自启动的办法

## rc.local

Linux 自带一个开机自启动的脚本，这是一个 shell 脚本，直接在里面写即可。不存在该文件则跳过该方法
  
- 编辑文件

```bash
sudo nano /etc/rc.local
```

默认内容如下,在`exit 0`前`#!/bin/sh -e`后写命令

```terminal
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.


exit 0
```

- 赋权，文件可能会没有可执行权限，输入`chmod +x /etc/rc.local`赋予权限

## system 服务启动

- 在`/etc/systemd/system/`目录下新建服务`demo.server`
- 编写`server`文件,自己要准备想自启动的 shell 脚本

```terminal
[Unit]
Description=            # 描述该服务的用途
After=network.target    # 在网络服务启动后运行

[Service]
User=                   # 运行该服务的用户，必填 echo ${SUDO_USER:-$(whoami)}
Type=simple             # 服务类型为 simple（默认）
ExecStart=              # 启动脚本的路径
ExecStop=               # 停止服务时运行的清理脚本
Restart=always          # 服务崩溃后总是重启
TimeoutStopSec=10       # 停止服务时等待的最长时间（10秒）
RestartSec=10s          # 重启前等待的时间（10秒）

[Install]
WantedBy=multi-user.target  # 该服务会在系统进入多用户模式（正常启动模式）时自动启动

```

- 更新 systemd 配置

```terminal
sudo systemctl daemon-reload
```

- 启用服务

```terminal
sudo systemctl enable demo.server
```

- 启动服务

```terminal
sudo systemctl start demo.server
```

- 其他相关命令

```bash
# 重启服务
sudo systemctl restart demo.service
# 停止服务
sudo systemctl stop demo.service
# 停用服务
sudo systemctl disable demo.service

```
