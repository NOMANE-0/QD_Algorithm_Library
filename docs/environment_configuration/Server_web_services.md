# 服务器网页服务部署

本文记录当前内网服务器的网页服务部署方式，并说明以后如何扩展新的网页入口。

本文中的“服务器”指主机名为 `31ge` 的 Ubuntu 服务器。在当前局域网中，它可以通过下面的域名访问：

```text
31ge.local
```

如果以后换到另一台服务器，只需要把文档中的 `31ge.local` 替换成新服务器的域名或 IP 地址即可。

当前部署目标：

- 首页导航：`http://31ge.local/`
- 网盘入口：`http://31ge.local/disk/`
- 算法库入口：`http://31ge.local/algo/`
- SSH 端口：建议迁移到 `2222`

## APT 包依赖

在一台新的 Ubuntu 服务器上，先更新软件源：

```bash
sudo apt update
```

安装基础依赖：

```bash
sudo apt install -y nginx git openssh-server curl
```

各软件用途：

| 软件包 | 用途 |
| --- | --- |
| `nginx` | 对外提供导航页、静态网页和反向代理 |
| `git` | 拉取和更新算法库仓库 |
| `openssh-server` | 提供 SSH 登录服务 |
| `curl` | 在服务器上测试本机 HTTP 服务 |

如果服务器启用了防火墙，还需要安装或确认 `ufw` 可用：

```bash
sudo apt install -y ufw
```

常用检查命令：

```bash
nginx -v
git --version
ssh -V
curl --version
```

`systemd` 是 Ubuntu Server 默认自带的服务管理器，一般不需要单独安装。

## 服务结构

服务器上主要有三类内容：

| 路径 | 用途 |
| --- | --- |
| `/var/www/nav/index.html` | 首页导航页 |
| `/home/qidian/QD_Algorithm_Library/docs/` | 算法库 docsify 文档 |
| `http://127.0.0.1:8080` | 网盘后端服务 |

Nginx 负责统一对外提供 80 端口：

- `/` 读取 `/var/www/nav`
- `/disk/` 反向代理到本机 `8080`
- `/algo/` 静态托管 docsify 文档目录

## Nginx 配置

配置文件：

```bash
sudo nano /etc/nginx/sites-available/31ge-nav
```

这里的 `31ge-nav` 只是配置文件名，含义是“31ge 服务器上的导航页配置”。如果换服务器，可以改成其他更合适的名字。

推荐配置如下：

```nginx
server {
    listen 80 default_server;
    listen [::]:80 default_server;

    server_name _;

    root /var/www/nav;
    index index.html;

    client_max_body_size 2048M;

    location / {
        try_files $uri $uri/ /index.html;
    }

    location /disk/ {
        proxy_pass http://127.0.0.1:8080;

        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }

    location /algo/ {
        alias /home/qidian/QD_Algorithm_Library/docs/;
        index index.html;
        try_files $uri $uri/ /algo/index.html;
    }
}
```

检查并重载：

```bash
sudo nginx -t
sudo systemctl reload nginx
```

`/algo/` 这里需要特别注意：

```nginx
try_files $uri $uri/ /algo/index.html;
```

不要写成：

```nginx
try_files $uri $uri/ /index.html;
```

否则访问 docsify 子路径时会退回首页导航页，而不是算法库的 `index.html`。

## 首页导航页

导航页文件：

```bash
sudo nano /var/www/nav/index.html
```

算法库入口示例：

```html
<a class="card" href="/algo/">
  <div class="title">算法库</div>
  <div class="desc">查看 QD Algorithm Library 文档。</div>
</a>
```

网盘入口示例：

```html
<a class="card" href="/disk/">
  <div class="title">进入网盘</div>
  <div class="desc">上传、下载文件。普通用户无删除权限。</div>
</a>
```

修改导航页后通常不需要重载 Nginx，刷新浏览器即可。

## 算法库 docsify 配置

算法库直接由 Nginx 静态托管，不需要长期运行：

```bash
docsify serve docs
```

docsify 入口文件：

```bash
/home/qidian/QD_Algorithm_Library/docs/index.html
```

由于 docsify 在子目录页面会尝试读取当前目录下的 `_navbar.md`，需要在 `window.$docsify` 中加入导航栏别名：

```js
alias: {
  '/.*/_navbar.md': '/_navbar.md',
},
```

否则访问下面这种页面时，顶部导航可能消失：

```text
http://31ge.local/algo/#/LearningPath/
```

正确访问方式：

```text
http://31ge.local/algo/
http://31ge.local/algo/#/LearningPath/
http://31ge.local/algo/#/LearningPath/Learning_path_1
```

## 文件权限

Nginx 需要能进入并读取 docsify 文件目录。

检查路径权限：

```bash
namei -l /home/qidian/QD_Algorithm_Library/docs/index.html
```

如果出现 `Permission denied`，执行：

```bash
chmod o+x /home/qidian
chmod o+x /home/qidian/QD_Algorithm_Library
chmod -R o+rX /home/qidian/QD_Algorithm_Library/docs
```

这些命令只允许 Nginx 读取网页文件，不会给其他用户 SSH 登录权限。

## 自动更新算法库

创建 systemd service：

```bash
sudo nano /etc/systemd/system/qd-algo-update.service
```

写入：

```ini
[Unit]
Description=Update QD Algorithm Library

[Service]
Type=oneshot
User=qidian
Group=qidian
WorkingDirectory=/home/qidian/QD_Algorithm_Library
ExecStart=/bin/bash -lc 'git fetch --prune origin main && git merge --ff-only origin/main'
```

创建 timer：

```bash
sudo nano /etc/systemd/system/qd-algo-update.timer
```

写入：

```ini
[Unit]
Description=Run QD Algorithm Library git pull every 10 minutes

[Timer]
OnBootSec=1min
OnUnitActiveSec=10min
Unit=qd-algo-update.service

[Install]
WantedBy=timers.target
```

启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now qd-algo-update.timer
```

查看状态：

```bash
systemctl status qd-algo-update.timer
systemctl list-timers | grep qd-algo
journalctl -u qd-algo-update.service -n 50
```

手动更新一次：

```bash
sudo systemctl start qd-algo-update.service
```

## SSH 端口改为 2222

先不要直接关闭 22 端口。建议先同时开放 `22` 和 `2222`，确认新端口能登录后，再关闭 `22`。

创建端口配置：

```bash
sudo nano /etc/ssh/sshd_config.d/99-port.conf
```

第一阶段写入：

```sshconfig
Port 22
Port 2222
```

检查并重载：

```bash
sudo sshd -t
sudo systemctl reload ssh
```

另开一个终端测试：

```bash
ssh -p 2222 qidian@31ge.local
```

确认能登录后，再把配置改成：

```sshconfig
Port 2222
```

再次检查并重载：

```bash
sudo sshd -t
sudo systemctl reload ssh
```

以后连接服务器：

```bash
ssh -p 2222 qidian@31ge.local
```

如果服务器启用了 `ufw`，需要先放行端口：

```bash
sudo ufw allow 2222/tcp
```

## 扩展新的网页

新增网页时，先判断它属于哪一种类型。下面示例中的 `example` 是占位名称，实际使用时替换成新网页或新服务的英文路径名。

### 静态网页

适用于纯 HTML、CSS、JS 文件，例如一个说明页、下载页或静态工具页。

假设新网页路径为：

```text
http://31ge.local/example/
```

创建目录：

```bash
sudo mkdir -p /var/www/nav/example
sudo nano /var/www/nav/example/index.html
```

如果只是普通静态网页，不需要新增 Nginx `location`，因为已有配置会从 `/var/www/nav` 读取文件：

```nginx
location / {
    try_files $uri $uri/ /index.html;
}
```

然后在 `/var/www/nav/index.html` 添加入口：

```html
<a class="card" href="/example/">
  <div class="title">示例网页</div>
  <div class="desc">这里写网页用途。</div>
</a>
```

### 反向代理服务

适用于已有后端服务，例如某个 Web 控制台运行在本机 `9000` 端口。

在 Nginx 的 `server { ... }` 中添加：

```nginx
location /example/ {
    proxy_pass http://127.0.0.1:9000;

    proxy_set_header Host $host;
    proxy_set_header X-Real-IP $remote_addr;
    proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    proxy_set_header X-Forwarded-Proto $scheme;

    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
}
```

然后检查并重载：

```bash
sudo nginx -t
sudo systemctl reload nginx
```

最后在导航页添加入口：

```html
<a class="card" href="/example/">
  <div class="title">示例服务</div>
  <div class="desc">这里写服务用途。</div>
</a>
```

### 新的 docsify 文档站

适用于另一个独立文档仓库。

假设文档目录为：

```text
/home/qidian/ExampleDocs/docs/
```

访问路径为：

```text
http://31ge.local/example-docs/
```

Nginx 配置：

```nginx
location /example-docs/ {
    alias /home/qidian/ExampleDocs/docs/;
    index index.html;
    try_files $uri $uri/ /example-docs/index.html;
}
```

注意 fallback 必须使用当前站点前缀：

```nginx
/example-docs/index.html
```

不要写成：

```nginx
/index.html
```

并且 docsify 的 `index.html` 中建议加入：

```js
alias: {
  '/.*/_navbar.md': '/_navbar.md',
},
```

## 常见问题

### 自动更新服务启动失败

查看日志：

```bash
systemctl status qd-algo-update.service --no-pager -l
journalctl -u qd-algo-update.service -n 80 --no-pager
```

如果日志中出现：

```text
Your local changes to the following files would be overwritten by merge
```

说明服务器仓库里有未提交的本地改动。先查看：

```bash
cd /home/qidian/QD_Algorithm_Library
git status --short
git diff
```

如果这些改动已经提交到远端，或者只是服务器上的临时热修补，可以先保存起来再更新：

```bash
git stash push -m "server local changes before auto update"
git fetch --prune origin main
git merge --ff-only origin/main
```

如果日志中出现 `HTTP/2`、`GnuTLS`、`TLS connection` 等错误，通常是服务器访问 GitHub 时网络不稳定。可以给仓库设置使用 HTTP/1.1：

```bash
cd /home/qidian/QD_Algorithm_Library
git config http.version HTTP/1.1
```

如果网络仍然不稳定，等待网络恢复后再次执行：

```bash
git fetch --prune origin main
git merge --ff-only origin/main
```

### 访问 `/algo/` 返回 500

查看错误日志：

```bash
sudo tail -n 80 /var/log/nginx/error.log
```

如果看到 `Permission denied`，检查路径权限：

```bash
namei -l /home/qidian/QD_Algorithm_Library/docs/index.html
```

如果看到 `rewrite or internal redirection cycle`，检查 `try_files` 是否写错。

### 子页面顶部导航栏消失

检查 docsify 是否配置了 `_navbar.md` 别名：

```js
alias: {
  '/.*/_navbar.md': '/_navbar.md',
},
```

### 页面刷新后还是旧内容

浏览器可能缓存了旧文件。先按：

```text
Ctrl + F5
```

如果仍然异常，打开浏览器开发者工具，清理该站点的缓存和 service worker。

### 修改 Nginx 后没有生效

确认执行过：

```bash
sudo nginx -t
sudo systemctl reload nginx
```

确认正在使用的配置文件：

```bash
readlink -f /etc/nginx/sites-enabled/31ge-nav
```
