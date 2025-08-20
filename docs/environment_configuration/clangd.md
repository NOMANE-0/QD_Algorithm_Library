# clangd

在模式 vscode 的 c++ 扩展提供的 intellisense ，如果项目结构简单的话还好，但一旦代码量一大，项目结构复杂一下，代码提示就变得缓慢（函数都打完了才弹出提示）甚至找不到函数直接报错找不到（编译能通过但是代码提示报红）

clangd 能提供快速的代码提示和代码纠正，但是使用 clangd 依赖于编译产物`compile_commands.json`来进行索引，想使用 clangd 就必须编译出它

## 安装

```bash
sudo apt install clangd
```

vscode 安装 clangd 扩展

![clangd扩展](images/clangd-image.png)

## 构建项目

在项目的`CMakeLists.txt`中添加下面一行即可,或者构建 cmake 时`cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1`

```terminal
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

clangd 默认会索引根目录（$SRC）和 build（$SRC/build）下的`compile_commands.json`，请确保文件在其中

生成完后在 vscode 中打开文件 clangd 就会自动索引了，第一次编译出来需要一段时间加载索引

## Q&A

Q：
出现`In included file: 'cmath' file not found`

导致部分文件索引不到

A：

```bash
sudo apt install libstdc++-12-dev
```

> [Clang: 'cmath' file not found](https://stackoverflow.com/questions/22752000/clang-cmath-file-not-found)
