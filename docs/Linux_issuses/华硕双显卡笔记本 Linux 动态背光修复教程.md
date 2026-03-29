# 华硕天选（TUF Gaming）Linux 动态背光控制修复全指南

### 1. 核心矛盾：为什么背光会失效？

华硕笔记本在 Linux 下的背光控制往往涉及两个驱动：核显驱动（`amdgpu_bl0`）和华硕 WMI 驱动（`nvidia_wmi_ec_backlight`）。

* **混合模式（Hybrid）**：系统默认优先加载 `nvidia_wmi_ec_backlight`。
* **集显模式（Integrated）**：NVIDIA 显卡断电，但该 WMI 驱动仍会抢占控制权。此时桌面环境（如 KDE）尝试调节一个断电硬件的接口，导致背光“假死”。

---

### 2. 方案演进：从“静态手动”到“动态自动”

在寻找终极方案前，我们先看如何通过手动配置实现单一模式下的背光修复。

### 阶段一：静态手动修复（适用于固定使用集显的用户）

如果你发现集显模式下无法调节背光，可以手动强制内核切换接口：

1. **修改 GRUB 引导参数**：
在 `/etc/default/grub` 的 `GRUB_CMDLINE_LINUX_DEFAULT` 中加入：
`acpi_backlight=native`
> **作用**：告诉内核跳过标准的 ACPI 亮度接口，直接使用显卡硬件自身的“原生”接口（如核显的 PWM 控制）。


2. **手动黑名单冲突模块**：
创建文件 `/etc/modprobe.d/asus-fix.conf` 并写入：
`blacklist nvidia_wmi_ec_backlight`
> **作用**：彻底禁用该模块，强制系统只能看到并使用 AMD 核显的背光接口。



---

### 阶段二：方案的局限性（为什么需要动态化？）

**问题点**：上述配置在“集显模式”下完美运行，但当你回到“混合模式”时，由于驱动被你彻底屏蔽了，背光控制又会瘫痪。

**目标**：我们需要一种机制，在**集显模式下屏蔽**它，在**混合模式下启用**它。

---

### 3. 终极解决方案：构建动态调度器

通过 `supergfxctl` 切换模式时，利用 Systemd 监听状态并自动“搬运”驱动模块。

### 第一步：清理静态残留

确保你已经撤销了阶段一的所有操作（恢复 GRUB、删除 blacklist 文件），否则动态脚本无法接管。

### 第二步：编写状态感知脚本

```bash
sudo nano /usr/local/bin/asus-backlight-toggle.sh

```

**代码实现与逻辑说明：**

```bash
#!/bin/bash
# 1. 从 supergfxd 配置文件中实时抓取显卡模式
MODE=$(grep -oP '"mode":\s*"\K[^"]+' /etc/supergfxd.conf)

# 2. 逻辑分支判断
if [ "$MODE" = "Integrated" ]; then
    # 【集显模式】
    # 作用：主动卸载 WMI 模块。这样 KDE 只能搜寻到唯一可用的 AMD 核显接口。
    modprobe -r nvidia_wmi_ec_backlight 2>/dev/null || true
elif [ "$MODE" = "Hybrid" ]; then
    # 【混合模式】
    # 作用：重新加载 WMI 模块。确保在双显卡并存时，能通过主板 WMI 正确调节。
    modprobe nvidia_wmi_ec_backlight 2>/dev/null || true
fi

```

赋予执行权限：`sudo chmod +x /usr/local/bin/asus-backlight-toggle.sh`

---

### 第三步：配置 Systemd 自动化监听

为了让系统感知到你点击了切换按钮，我们需要监控 `/etc/supergfxd.conf` 的变化。

1. **创建 Service 任务** (`/etc/systemd/system/asus-backlight.service`):

```ini
[Unit]
Description=ASUS Backlight Toggle Service
[Service]
Type=oneshot
ExecStart=/usr/local/bin/asus-backlight-toggle.sh

```

2. **创建 Path 监听器** (`/etc/systemd/system/asus-backlight.path`):

```ini
[Unit]
Description=Watch supergfxd mode changes
[Path]
PathModified=/etc/supergfxd.conf
[Install]
WantedBy=multi-user.target

```

---

## 4. 方案评估

| 方案 | 优势 | 劣势/风险 | 置信度 |
| --- | --- | --- | --- |
| **静态配置** | 简单直接，适合不常切换显卡的用户。 | 切换显卡模式后必须手动修改系统配置并重启。 | 高 (针对单一模式) |
| **动态脚本** | **全自动**，无需重启系统，仅需注销桌面或重启 KWin。 | 依赖 `supergfxd` 的配置文件路径。 | **极高 (生产环境推荐)** |

## 5. 快速启用

执行以下命令，让配置立即生效：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now asus-backlight.path

```
