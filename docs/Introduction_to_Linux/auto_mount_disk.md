# 💻 Linux 开机自动挂载 Windows 硬盘教程

这个教程教你如何设置 Linux 系统，让它在每次启动时自动识别并挂载你的 Windows 硬盘分区，方便你直接访问 Windows 下的文件。

>! 确保你在 Windows 中禁用了**快速启动（Fast Startup）**和**休眠（Hibernate）**，否则 Linux 可能无法安全地写入该分区。

## 步骤一：找到 Windows 硬盘的“身份证” (UUID)

为了稳定地挂载分区，我们需要使用它的唯一标识符 **UUID**（Universal Unique Identifier），就像分区的身份证号码一样。

1.  **打开终端：**
    按下 `Ctrl + Alt + T` 打开终端。

2.  **执行命令：**
    输入以下命令并按回车：

    ```bash
    sudo blkid
    ```

3.  **识别并记录信息：**
    在输出结果中，你需要找到你的 Windows 分区。它通常具有 `TYPE="ntfs"` 的属性，并且有一个你认识的 `LABEL`（如 Windows 或 Data）。

      * **重点关注并记录以下信息：**
          * **UUID：** 一串很长的数字和字母组合。
          * **TYPE：** 应该是 `ntfs` 或 `ntfs-3g`。

    > **💡 示例片段：**
    > `/dev/nvme0n1p2: BLOCK_SIZE="512" UUID="821C044F1C044121" TYPE="ntfs" PARTLABEL="Basic data partition" PARTUUID="ea88149d-1ee6-4425-801d-a1a441cc85d2"`

## 步骤二：创建挂载点（像房子的门牌号）

我们需要在 Linux 文件系统中创建一个空目录，作为 Windows 硬盘的“入口”或“挂载点”。

1.  **选择一个位置：**
    我们通常在 `/mnt` 目录下创建挂载点。

2.  **创建目录：**
    请替换 `My_Windows` 为你想要的名字：

    ```bash
    sudo mkdir /mnt/My_Windows
    ```

## 步骤三：编辑配置文件 `/etc/fstab`

`/etc/fstab` 是 Linux 系统中用于定义文件系统挂载信息的配置文件。

1.  **打开配置文件：**
    使用 `nano` 编辑器打开文件。

    ```bash
    sudo nano /etc/fstab
    ```

2.  **添加挂载配置行：**
    在文件的最末尾，另起一行，**复制并修改**下面的配置行：

    ```fstab
    UUID=A4B8C0D4B8C0D4B8   /mnt/My_Windows   ntfs-3g   defaults,windows_names,locale=zh_CN.utf8,uid=1000,gid=1000,umask=0022   0   0
    ```

    **📌 配置说明：**

    | 配置项 | 你的替换内容 | 目的 |
    | :--- | :--- | :--- |
    | **`UUID`** | 替换为你记录的 **UUID**。 | 告诉系统要挂载哪个分区。 |
    | **`/mnt/...`** | 替换为你创建的**挂载点** (`/mnt/My_Windows`)。 | 告诉系统挂载到哪里。 |
    | **`ntfs-3g`** | 保持不变。 | 使用 `ntfs-3g` 驱动来支持读写 NTFS 分区。 |
    | **`uid=1000,gid=1000`** | 保持不变（除非你的用户 ID 不是 1000）。 | 让你当前的 Linux 用户有权限读写该分区。 |
    | **`locale=zh_CN.utf8`** | 保持不变。 | 确保中文文件名可以正常显示。 |
    | **`0   0`** | 保持不变。 | 告诉系统在启动时不进行备份或文件系统检查。 |

3.  **保存并退出：**

      * 在 `nano` 中，按 `Ctrl + O` (保存/Write Out)，然后按回车。
      * 按 `Ctrl + X` (退出/Exit)。

## 步骤四：测试配置是否成功

在不重启的情况下，我们可以使用一个命令立即测试 `/etc/fstab` 的配置是否生效。

1.  **执行测试命令：**

    ```bash
    sudo mount -a
    ```

      * 如果命令执行后**没有输出任何错误信息**，说明配置成功！

2.  **验证挂载：**
    你可以查看挂载点的内容：

    ```bash
    ls /mnt/My_Windows
    ```

    你应该能看到 Windows 硬盘里的文件列表。

