# SSH

ssh是一种加密的网络通信协议，常用远程操控主机

## 被控端环境

ssh客户端一般都自动装了，而服务端则需要自己安装。

```terminal
# 安装服务端
sudo apt install openssh-server

# 一般装了后就一直启动了，少部分不会自己启动
# 查看ssh服务状态
# 如果运行该命令时提示 "找不到 ssh.service 单元"，则需要安装 SSH 服务
sudo systemctl status ssh

```

`sudo systemctl status ssh`输出如下

```terminal
ssh.service - OpenBSD Secure Shell server
     Loaded: loaded (/lib/systemd/system/ssh.service; enabled; preset: enabled)
     Active: active (running) since Wed 2024-05-08 07:52:50 UTC; 24min ago
TriggeredBy: ● ssh.socket
       Docs: man:sshd(8)
             man:sshd_config(5)
    Process: 637 ExecStartPre=/usr/sbin/sshd -t (code=exited, status=0/SUCCESS)
   Main PID: 657 (sshd)
      Tasks: 1 (limit: 3420)
     Memory: 5.5M
        CPU: 398ms
     CGroup: /system.slice/ssh.service
             └─657 "sshd: /usr/sbin/sshd -D [listener] 0 of 10-100 startups"

May 08 07:52:49 radxa-zero systemd[1]: Starting ssh.service - OpenBSD Secure Shell server...
May 08 07:52:50 radxa-zero sshd[657]: Server listening on :: port 22.
May 08 07:52:50 radxa-zero systemd[1]: Started ssh.service - OpenBSD Secure Shell server.
May 08 07:53:22 radxa-zero sshd[783]: Accepted password for radxa from 192.168.31.1 port 49474 ssh2
May 08 07:53:22 radxa-zero sshd[783]: pam_unix(sshd:session): session opened for user radxa(uid=1000) by (uid=0)
May 08 07:53:22 radxa-zero sshd[783]: pam_env(sshd:session): deprecated reading of user environment enabled
```

- 如果 `Active` 状态为 `active (running)`，则 SSH 处于运行状态。
- 如果 `Active` 状态为 `inactive (dead)`，则 SSH 已停止，即服务未运行，需要重新启动。

```terminal
# 启动ssh服务
sudo systemctl restart ssh

# 开机自启ssh服务
sudo systemctl enable --now ssh
```

## ssh连接

想要ssh远程连接，需要知道被控端的用户名`$USER`和地址 `ip`

- `$USER`用户名获取：在被控端终端输入`echo $USER`，输入的结果就是用户名
- `ip`地址获取：在被控端终端输入`ip a`，在输出的信息中查找网络接口（如 `eth0`、`ens33` 或 `wlan0`），地址通常以 `inet` 开头（如 `inet 192.168.1.100/24`），`192.168.1.100`就是ip地址

- ssh连接：在主机终端输入`ssh $USER@ip`，然后会要求你输入被控端的密码，输入即可连接

第一次连接会弹出以下信息，输入`yes`即可

```terminal
The authenticity of host 'gitee.com (180.76.198.225)' can't be established.
ED25519 key fingerprint is SHA256:+ULzij2u99B9eWYFTw1Q4ErYG/aepHLbu96PAUCoV88.
This key is not known by any other names
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```



## ssh密钥

每次连接都要输入密码，很麻烦，为了免密连接就需要ssh密钥了

密钥分成公钥和私钥，公钥是给别人的，私钥是自己独有，泄露了相当于泄露密码了

```terminal
# 生成ssh密钥，一直 Enter 即可
ssh-keygen -t rsa

# 查看ssh公钥
cat ~/.ssh/id_rsa.pub

# 复制公钥到其他主机
ssh-copy-id $USER@ip
```

当然也可以手动复制公钥过去，将公钥上传到被控端的`~/.ssh/authorized_keys `里，文件不存在的话自己创建一个



