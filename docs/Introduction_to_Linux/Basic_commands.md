# 基础命令

## 文件与目录操作

- `cd`    改变当前目录环境

```terminal
# 返回用户目录
cd 
cd ~

# 到上一家目录
cd ..

# 到根目录
cd /
```

- `rm`    删除文件

```terminal
# 删除单个文件
rm file.txt

# 递归删除目录
rm -r dir

# 强制删除
rm -f

# 完全删除文件，不进回收站
rm -rf dir 
```

- `cp`    复制文件，兼重命名

```terminal
# 复制文件
cp file1 file2

# 复制文件夹，如果dir2不存在则重命名
cp -r dir1 dir2
```

- `mv`    移动文件,兼重命名

```terminal
# 移动文件
mv old.txt new.txt

# 移动文件夹，并不用 -r
mv dir /target/path
```

- `ls`    列出目录内容

```terminal
# 列出目录内容
ls [路径]

# 列出详细列表
ls -l [路径]

# 显示隐藏文件
ls -a [路径]
```

- `touch`    创建文件

```terminal
touch file.txt
```

- `mkdir`    创建文件夹

```terminal
# 创建目录
mkdir dir

# 自动化创建父目录，不存在parant的话会自动创建
mkdir -p parent/child
```

## 文本编辑与查看

- `cat`    查看文件内容

```terminal
# 在终端把文件内容打印出来
cat file.txt

# 合并文件
cat file1 file2 > merged.txt 
```

- `vim / vi`    文本编辑器
  
  - `vim file`：编辑文件
  
  - 安装`vim`：`sudo apt install vim` 
  
  - 安装`vi`:默认安装了
  
  - 编辑：输入<kbd>i</kbd>进入编辑模式
  
  - 退出：先按<kbd>Esc</kbd>，然后依次输入<kbd>:wq</kbd>（保存并退出）或<kbd>:q</kbd>（退出不保存）,最后回车<kbd>Enter</kbd>退出

- `nano`    文本编辑器
  
  - `nano file`：编辑文件
  - 退出：<kbd>Alt</kbd>+<kbd>x</kbd>，然后输入<kbd>y</kbd>（保存）或<kbd>n</kbd>（不保存），保存的话还要再回车<kbd>Enter</kbd>退出

- `grep`     文本搜索

## 权限与系统管理

- `sudo`    以超级用户权限执行命令

可以在任何命令前加入，代表以`root`用户运行

- `chmod`    修改文件权限

```terminal
# 赋予文件可执行权限
sudo chmod +x file.sh

# 赋予文件最高权限
sudo chmod 777 file
```

- `chown`    修改文件所有者

```terminal
sudo chown $USER file
```

## 软件包管理

- `apt`    包管理工具

```terminal
# 更新软件源
sudo apt update

# 安装软件
sudo apt install <package>

# 卸载软件
sudo apt remove <package>
# 彻底卸载软件，会删到其他依赖（危险）
sudo apt remove <package> --purge
```

- `dpkg`    `.deb`包安装

```terminal
# 安装
sudo dpkg -i package.deb

# 卸载
sudo dpkg -r package
```

## 压缩与归档

- `tar`    打包/解压`.tar.gz`文件

```terminal
# 压缩 把 dir/ 目录压缩成 archive.tar.gz
tar -czvf archive.tar.gz dir/

# 解压
tar -zxvf archive.tar.gz
```

- `unzip`    解压`.zip`文件

```terminal
# 使用前需安装
sudo apt install unzip

# 解压
unzip file.zip
```

> - 在以上的使用中，都可以使用  \*.txt 这种形式来指代全部txt文件
> 
> - 每个命令都有许多的命令用法，这里只列出常用的命令用法