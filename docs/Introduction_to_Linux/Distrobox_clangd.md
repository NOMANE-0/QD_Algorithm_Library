# Distrobox 与 clangd 项目环境配置

> 目标：为一个 C++ / ROS 项目提供可复现的开发环境，并让 VS Code 的 clangd 使用容器内工具链完成代码提示。

## 适用场景

项目依赖较多、不同电脑系统环境不一致时，可以把编译和运行环境放进 Distrobox。
Distrobox 相比传统 Docker 开发容器更贴近日常 Linux 使用方式：容器用户、主机 home、
图形界面、USB/显卡设备等都更容易打通。

推荐原则：

- 编译、运行、测试首选在 Distrobox 内执行。
- VS Code 可以继续在主机打开项目目录。
- clangd 通过包装脚本进入 Distrobox，使用容器内的头文件、编译器和系统库。
- 项目根目录提供 `distrobox.ini` 和 `scripts/`，让新电脑可以一条命令创建环境。

## 安装 Distrobox

Ubuntu 上推荐先安装 Podman 和 Distrobox：

```bash
sudo apt update
sudo apt install podman distrobox
```

确认安装成功：

```bash
distrobox --version
podman --version
```

## 项目目录约定

假设项目结构如下：

```text
my_project/
├── distrobox.ini
├── scripts/
│   ├── bootstrap_distrobox.sh
│   ├── clangd_distrobox.sh
│   └── setup_ubuntu_mirror.sh
├── .vscode/
│   └── settings.json
├── src/
└── README.md
```

容器内建议使用固定工作区路径，例如：

```text
/workdir
```

ROS 项目也可以使用习惯路径：

```text
/ros_ws
```

路径固定后，README、脚本、clangd path mapping 都更稳定。

## distrobox.ini

在项目根目录创建 `distrobox.ini`：

```ini
[my_project]
image=ubuntu:22.04
hostname=my_project
home=~/.local/share/distrobox/my_project
init=false
nvidia=false
pull=true
start_now=false
pre_init_hooks="bash /workdir/scripts/setup_ubuntu_mirror.sh"
additional_packages="sudo bash-completion ca-certificates curl gnupg lsb-release git pkg-config build-essential cmake gdb clangd"
additional_flags="--device /dev/dri --device /dev/bus/usb --group-add keep-groups"
init_hooks="bash /workdir/scripts/bootstrap_distrobox.sh"
volume="${PWD}:/workdir:rw"
```

常用字段说明：

- `image`：基础系统镜像。ROS2 Humble 通常用 `ubuntu:22.04`。
- `home`：容器 home 目录。每个项目单独一个 home，避免不同项目污染。
- `pre_init_hooks`：初始化前执行，适合换 apt 源。
- `additional_packages`：创建容器时先安装的基础包。
- `additional_flags`：额外容器参数，例如显卡、USB、保留用户组。
- `init_hooks`：容器创建后执行，适合安装项目依赖。
- `volume`：把当前项目挂载到容器固定路径。

创建容器：

```bash
distrobox assemble create --file distrobox.ini
```

进入容器：

```bash
distrobox enter my_project
cd /workdir
```

## 换 apt 源脚本

创建 `scripts/setup_ubuntu_mirror.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script inside the Distrobox container with sudo/root." >&2
    exit 1
fi

mirror="${PROJECT_APT_MIRROR:-http://mirrors.ustc.edu.cn/ubuntu/}"
arch="$(dpkg --print-architecture)"

if [[ "${arch}" == "arm64" || "${arch}" == "armhf" ]]; then
    mirror="${PROJECT_APT_MIRROR:-http://mirrors.ustc.edu.cn/ubuntu-ports/}"
fi

cat >/etc/apt/sources.list <<EOF
deb ${mirror} jammy main restricted universe multiverse
deb ${mirror} jammy-updates main restricted universe multiverse
deb ${mirror} jammy-backports main restricted universe multiverse
deb ${mirror} jammy-security main restricted universe multiverse
EOF

rm -rf /var/lib/apt/lists/*
echo "Ubuntu apt mirror set to: ${mirror}"
```

给脚本执行权限：

```bash
chmod +x scripts/setup_ubuntu_mirror.sh
```

## 依赖安装脚本

创建 `scripts/bootstrap_distrobox.sh`。这个脚本负责安装项目依赖，并把常用环境变量写入
`/etc/profile.d/`，让每次进入容器自动生效。

```bash
#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script inside the Distrobox container with sudo/root." >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive

marker=/var/lib/my-project-bootstrap-v1
if [[ -f "${marker}" && "${PROJECT_BOOTSTRAP_FORCE:-0}" != "1" ]]; then
    echo "my_project Distrobox dependencies are already bootstrapped."
    echo "Set PROJECT_BOOTSTRAP_FORCE=1 to reinstall."
    exit 0
fi

apt-get update
apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    clangd \
    cmake \
    curl \
    g++ \
    gdb \
    git \
    libopencv-dev \
    pkg-config

cat >/etc/profile.d/my_project.sh <<'EOF'
# my_project Distrobox defaults.
export PROJECT_WORKDIR=/workdir
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    . /opt/ros/humble/setup.bash
fi

if [ -f /workdir/install/setup.bash ]; then
    # shellcheck disable=SC1091
    . /workdir/install/setup.bash
fi
EOF

apt-get clean
rm -rf /var/lib/apt/lists/*
touch "${marker}"
```

给脚本执行权限：

```bash
chmod +x scripts/bootstrap_distrobox.sh
```

重新执行依赖安装：

```bash
distrobox enter my_project -- sudo PROJECT_BOOTSTRAP_FORCE=1 /workdir/scripts/bootstrap_distrobox.sh
```

## clangd 包装脚本

VS Code 在主机运行，但代码提示需要容器内的系统头文件和依赖。做法是让 VS Code 调用一个
主机脚本，这个脚本再进入 Distrobox 执行 `/usr/bin/clangd`。

创建 `scripts/clangd_distrobox.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd -P)"

path_mappings="${workspace_dir}=/run/host${workspace_dir},${workspace_dir}=/workdir"

exec distrobox enter --no-tty --name my_project -- \
    /usr/bin/clangd "--path-mappings=${path_mappings}" "$@"
```

给脚本执行权限：

```bash
chmod +x scripts/clangd_distrobox.sh
```

`--path-mappings` 很关键。Distrobox 内部通常可以通过 `/run/host/...` 访问主机路径，
而项目又挂载到了 `/workdir`。clangd 需要知道这些路径其实是同一份代码，否则跳转、
诊断和 compile commands 可能对不上。

## VS Code 配置

创建 `.vscode/settings.json`：

```json
{
    "clangd.path": "${workspaceFolder}/scripts/clangd_distrobox.sh",
    "clangd.arguments": [
        "--query-driver=/usr/bin/g++,/usr/bin/c++"
    ],
    "C_Cpp.intelliSenseEngine": "disabled"
}
```

说明：

- `clangd.path`：指定使用包装脚本。
- `--query-driver`：允许 clangd 查询容器内编译器的默认 include path。
- `C_Cpp.intelliSenseEngine`：关闭 Microsoft C/C++ 插件的 IntelliSense，避免和 clangd 冲突。

如果项目 `.gitignore` 忽略了整个 `.vscode/`，但希望提交 `settings.json`，可以这样写：

```gitignore
.vscode/
!.vscode/
!.vscode/settings.json
.vscode/*
!.vscode/settings.json
```

## 编译数据库

clangd 最好配合 `compile_commands.json` 使用。普通 CMake 项目：

```bash
distrobox enter my_project
cd /workdir
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -sf build/compile_commands.json compile_commands.json
```

ROS2 / colcon 项目：

```bash
distrobox enter my_project
cd /workdir
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

如果内存有限，同时限制 colcon 包并行数和单包内部编译线程：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --symlink-install --parallel-workers 4
```

## ROS 项目的额外建议

ROS 项目常见工作区路径是 `/ros_ws`，可以把上面的 `/workdir` 全部替换为 `/ros_ws`。

为了避免 ROS 日志写到 Distrobox home 后出现权限问题，可以在 profile 中固定日志目录：

```bash
export ROS_HOME=/ros_ws/.ros
export ROS_LOG_DIR=/ros_ws/log/ros
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
```

如果项目依赖本地 `.so`，例如相机 SDK，也应该把运行时库路径加入 profile：

```bash
export LD_LIBRARY_PATH=/ros_ws/src/rm_utils/hikSDK/lib/amd64:${LD_LIBRARY_PATH:-}
```

## 常见问题

### VS Code 没有代码提示

先确认容器能进入：

```bash
distrobox enter my_project -- /usr/bin/clangd --version
```

再确认脚本可执行：

```bash
./scripts/clangd_distrobox.sh --version
```

最后在 VS Code 中执行 `clangd: Restart language server`。

### clangd 找不到头文件

检查是否生成了 `compile_commands.json`，以及项目是否已经在容器内成功配置或构建过。

ROS 项目需要 source 环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

这些命令建议写进 `/etc/profile.d/<project>.sh`。

### Distrobox 里混入了主机 conda / pixi 环境

如果构建日志里出现主机路径，例如 `.pixi/envs/default` 或 `conda`，说明环境变量被带进了容器。
可以用干净环境执行构建：

```bash
distrobox enter --name my_project -- env -i \
    HOME="$HOME" USER="$USER" LOGNAME="$USER" SHELL=/bin/bash \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc -lc '
        cd /workdir
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install --parallel-workers 4
    '
```

更根本的做法是检查 `~/.bashrc`、`~/.profile` 中是否自动激活了 conda/pixi。

### ROS launch 无法创建日志目录

如果报错类似：

```text
PermissionError: [Errno 13] Permission denied: '<home>/.ros/log'
```

修复当前目录权限：

```bash
sudo chown -R "$USER:$USER" "$HOME/.ros"
chmod 700 "$HOME/.ros"
```

并建议固定日志目录到工作区：

```bash
export ROS_HOME=/ros_ws/.ros
export ROS_LOG_DIR=/ros_ws/log/ros
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
```

## 最小闭环

一个项目完成 Distrobox + clangd 配置后，至少检查下面几项：

```bash
# 1. 容器能进入
distrobox enter my_project

# 2. 编译器和 clangd 存在
g++ --version
clangd --version

# 3. 项目能构建
CMAKE_BUILD_PARALLEL_LEVEL=4 cmake --build build -j4

# 4. VS Code 使用的是容器 clangd
./scripts/clangd_distrobox.sh --version
```

ROS 项目则使用：

```bash
distrobox enter my_project
cd /workdir
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --symlink-install --parallel-workers 4
```

