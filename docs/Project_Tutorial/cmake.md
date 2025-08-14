cmake 是⼀种常⽤、⽅便的，⽤于组织 Linux 下 C++ 程序的⼯具。有许多库，例如 OpenCV、g2o、

Ceres 等，都⽤ cmake 组织它们的⼯程。所以，不管是使⽤别⼈的库，还是编写⾃⼰的库，都需要掌握

⼀些 cmake 的基本知识。
本教程基于**[Cmake Practice.pdf](http://file.ncnynl.com/ros/CMake%20Practice.pdf)**（cmake 实践，由⼀位北⼤同学撰写）
为了阅读方便，本文档整理摘抄并修改自互联网上的PDF版本，仅供学习参考使用。

# ❗**学习要求：**

请阅读此⽂的第 1 ⾄ 6 章，并完成以下⼯作：

书写⼀个由 cmake 组织的 C++ ⼯程，要求如下：

1. include/hello.h 和 src/hello.c 构成了 libhello.so 库。hello.c 中提供⼀个函数 sayHello()，调⽤此函数时往屏幕输出⼀⾏“Hello World”。我们已经为你准备了 hello.h 和 hello.c 这两个代码在下方。

2. ⽂件 useHello.c 中含有⼀个 main 函数，它可以编译成⼀个可执⾏⽂件，名为“sayhello”

3. 默认⽤ Release 模式编译这个⼯程。

4. 如果⽤户使⽤ sudo make install，那么将 hello.h 放⾄/usr/local/include/下，将 libhello.so 放⾄/usr/local/lib/下。

请按照上述要求组织源代码⽂件，并书写 CMakeLists.txt。

```C++
#pragma once
void sayHello(); 
```

```C++
#include "hello.h"
#include <iostream>

void sayHello() {std::cout<<"Hello World"<<std::endl;}
```

```C++
#include "hello.h"

int main( int argc, char** argv ) {
  sayHello();
}

```

# 一、初识Cmake

略

# 二、安装Cmake

略

# 三、初试cmake - cmake 的helloworld

本节选择了一个最简单的例子 Helloworld 来演练一下 cmake 的完整构建过程，本节并不

会深入的探讨 cmake，仅仅展示一个简单的例子，并加以粗略的解释。

## 1.准备工作

首先，在 `/backup` 目录建立一个 `cmake` 目录，用来放置我们学习过程中的所有练习。

```Shell
mkdir -p /backup/cmake
```

以后我们所有的 cmake 练习都会放在/backup/cmake 的子目录下(你也可以自行安排目录，这个并不是限制，仅仅是为了叙述的方便)。

然后在 cmake 建立第一个练习目录 `t1`：

```Shell
cd /backup/cmake
mkdir t1
cd t1
```

在 t1 目录建立 `main.c` 和 `CMakeLists.txt`(注意文件名大小写)：

`main.c` 文件内容：

```Shell
// main.c
#include <stdio.h>
int main()
{
    printf("Hello World from t1 Main!\n");
    return 0;
}
```

`CmakeLists.txt` 文件内容：

```Shell
PROJECT(HELLO)
SET(SRC_LIST main.c)
MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${HELLO_SOURCE_DIR})
ADD_EXECUTABLE(hello ${SRC_LIST})
```

## 2.开始构建

所有的文件创建完成后，`t1` 目录中应该存在 `main.c` 和 `CMakeLists.txt` 两个文件。接下来我们来构建这个工程，在这个目录运行：

`cmake .` (注意命令后面的点号，代表本目录)。·

输出大概是这个样子：

```Shell
-- Check for working C compiler: /usr/bin/gcc
-- Check for working C compiler: /usr/bin/gcc -- works
-- Check size of void*
-- Check size of void* - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- This is BINARY dir /backup/cmake/t1
-- This is SOURCE dir /backup/cmake/t1
-- Configuring done
-- Generating done
-- Build files have been written to: /backup/cmake/t1
```

再让我们看一下目录中的内容，你会发现，系统自动生成了：

`CMakeFiles`，`CMakeCache.txt`，`cmake_install.cmake` 等文件，并且生成了 `Makefile`。

然后进行工程的**实际构建**，在这个目录输入 `make` 命令，大概会得到如下的彩色输出：

```Shell
Scanning dependencies of target hello
[100%] Building C object CMakeFiles/hello.dir/main.o
Linking C executable hello
[100%] Built target hello
```

如果你需要看到 `make` 构建的详细过程，可以使用 `make VERBOSE=1` 或者 `VERBOSE=1 make` 命令来进行构建。

这时候，我们需要的目标文件 hello 已经构建完成，位于当前目录，尝试运行一下：

```Shell
./hello
```

得到输出：

```Shell
Hello World from t1 Main!
```

恭喜您，到这里为止您已经完全掌握了 cmake 的使用方法。



## 3.简单解释

我们来重新看一下 `CMakeLists.txt`，这个文件是 cmake 的构建定义文件，文件名是大小写相关的，如果工程存在多个目录，需要确保每个要管理的目录都存在一个 `CMakeLists.txt`。(关于多目录构建，后面我们会提到，这里不作过多解释)。

上面例子中的 CMakeLists.txt 文件内容如下：

```Shell
PROJECT (HELLO)
SET(SRC_LIST main.c)
MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir "${HELLO_SOURCE_DIR})
ADD_EXECUTABLE(hello ${SRC_LIST})
```



### *PROJECT 指令的语法*

`PROJECT(projectname [CXX] [C] [Java])`

你可以用这个指令定义工程名称，并可指定工程支持的语言，支持的语言列表是可以忽略的，默认情况表示支持所有语言。这个指令隐式的定义了两个 cmake 变量：

`<projectname>_BINARY_DIR` 以及 `<projectname>_SOURCE_DIR`，这里就是 `HELLO_BINARY_DIR` 和 `HELLO_SOURCE_DIR` (所以 `CMakeLists.txt` 中两个 `MESSAGE` 指令可以直接使用了这两个变量)，因为采用的是内部编译，两个变量目前指的都是工程所在路径`/backup/cmake/t1`，后面我们会讲到外部编译，两者所指代的内容会有所不同。

同时 cmake 系统也帮助我们预定义了 `PROJECT_BINARY_DIR` 和 `PROJECT_SOURCE_DIR` 变量，他们的值分别跟 `HELLO_BINARY_DIR` 与 `HELLO_SOURCE_DIR` 一致。

为了统一起见，建议以后直接使用`PROJECT_BINARY_DIR`和`PROJECT_SOURCE_DIR`，即使修改了工程名称，也不会影响这两个变量。如果使用了`<projectname_SOURCE_DIR>`，修改工程名称后，需要同时修改这些变量。



### *SET指令的语法*

`SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])`

现阶段，你只需要了解 SET 指令可以用来显式的定义变量即可。比如我们用到的是`SET(SRC_LIST main.c)`，如果有多个源文件，也可以定义成：`SET(SRC_LIST main.c t1.c t2.c)`。



### *MESSAGE指令的语法*

`MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display" ...)`

这个指令用于向终端输出用户定义的信息，包含了三种类型：

1. `SEND_ERROR`，产生错误，生成过程被跳过

2. `STATUS`，输出前缀为`--`的信息

3. `FATAL_ERROR`，立即终止所有cmake过程

我们在这里使用的是 `STATUS` 信息输出，演示了由 `PROJECT` 指令定义的两个隐式变量 `HELLO_BINARY_DIR` 和 `HELLO_SOURCE_DIR`。



### *ADD_EXECUTABLE指令的语法*

**`ADD_EXECUTABLE(hello ${SRC_LIST})`**

定义了这个工程会生成一个文件名为 hello 的可执行文件，相关的源文件是 SRC_LIST 中定义的源文件列表， 本例中你也可以直接写成`ADD_EXECUTABLE(hello main.c)`。

在本例我们使用了`${}`来引用变量，这是 cmake 的变量应用方式，但是，有一些例外，比如在 `IF 控制语句`，变量是直接使用变量名引用，而不需要`${}`。如果使用了`${}`去应用变量，其实 IF 会去判断名为`${}`所代表的值的变量，那当然是不存在的了。

将本例改写成一个最简化的`CMakeLists.txt`：

```Shell
PROJECT(HELLO)
ADD_EXECUTABLE(hello main.c)
```



## 4.基本语法规则

前面提到过，cmake 其实仍然要使用”cmake 语言和语法”去构建，上面的内容就是所谓的 ”cmake 语言和语法”，最简单的语法规则是：

1. 变量使用`${}`方式取值，但是在 IF 控制语句中是直接使用变量名。

2. 指令(参数1 参数2...)

   参数使用括弧括起，参数之间使用空格或分号分开。

   以上面的 `ADD_EXECUTABLE` 指令为例，如果存在另外一个 func.c 源文件，就要写成：

   `ADD_EXECUTABLE(hello main.c func.c`)或者

   `ADD_EXECUTABLE(hello main.c;func.c)`

3. 指令是大小写无关的，参数和变量是大小写相关的。但，推荐你全部使用大写指令。

   上面的 MESSAGE 指令已经用到了这条规则：

   `MESSAGE(STATUS “This is BINARY dir” ${HELLO_BINARY_DIR})`

   也可以写成：

   `MESSAGE(STATUS “This is BINARY dir ${HELLO_BINARY_DIR}”)`

**这里需要特别解释的是作为工程名 HELLO 和生成的可执行文件 hello 是没有任何关系的。hello 定义的可执行文件的文件名，你完全可以写成：`ADD_EXTCUTABLE(t1 main.c)`，编译后会生成一个 t1 可执行文件。**

 

## 5.关于语法的疑惑

cmake 的语法还是比较灵活而且考虑到各种情况，比如 `SET(SRC_LIST main.c)` 也可以写成 `SET(SRC_LIST "main.c")` 是没有区别的，但是假设一个源文件的文件名是 `fu nc.c`(文件名中间包含了空格)。 这时候就必须使用双引号，如果写成了SET(SRC_LIST fu nc.c)，就会出现错误，提示你找不到 `fu 文件`和 `nc.c 文件`。这种情况，就必须写成：`SET(SRC_LIST "fu nc.c")`。



此外，你可以可以忽略掉 source 列表中的源文件后缀，比如可以写成 `ADD_EXECUTABLE(t1 main)`，cmake 会自动的在本目录查找 `main.c` 或者 `main.cpp` 等，当然，最好不要偷这个懒，以免这个目录确实存在一个 `main.c` 一个 `main`。



同时参数也可以使用分号来进行分割。

下面的例子也是合法的：

`ADD_EXECUTABLE(t1 main.c t1.c)` 可以写成 `ADD_EXECUTABLE(t1 main.c;t1.c)`。

我们只需要在编写 `CMakeLists.txt` 时注意形成统一的风格即可。



## 6.清理工程

跟经典的 autotools 系列工具一样，运行：

```Shell
make clean
```

即可对构建结果进行清理。



## 7.问题？问题！

“我尝试运行了 `make distclean`，这个指令一般用来清理构建过程中产生的中间文件的，如果要发布代码，必然要清理掉所有的中间文件，但是为什么在 cmake 工程中这个命令是无效的?”

是的，cmake 并不支持 `make distclean`，关于这一点，官方是有明确解释的：

> 因为 `CMakeLists.txt` 可以执行脚本并通过脚本生成一些临时文件，但是却没有办法来跟踪这些临时文件到底是哪些。因此，没有办法提供一个可靠的 make distclean 方案。



> Some build trees created with GNU autotools have a "make distclean" target that cleans the build and also removes Makefiles and other parts of the generated build system. CMake does not generate a "make distclean" target because `CMakeLists.txt` files can run scripts and arbitrary commands; CMake has no way of tracking exactly which files are generated as part of running CMake. Providing a distclean target would give users the false impression that it would work as expected. (CMake does generate a "make clean" target to remove files generated by the compiler and linker.)

> A "make distclean" target is only necessary if the user performs an in-source build. CMake supports in-source builds, but we strongly encourage users to adopt the notion of an out-of-source build. Using a build tree that is separate from the source tree will prevent CMake from generating any files in the source tree. Because CMake does not change the source tree, there is no need for a distclean target. One can start a fresh build by deleting the build tree or creating a separate build tree.



同时，还有另外一个非常重要的提示，就是：我们刚才进行的是内部构建(in-source build)，而 cmake 强烈推荐的是外部构建(out-of-source build)。



## 8.内部构建与外部构建

在使用构建系统（如 CMake）时，**内部构建**会在源代码目录生成大量中间文件，难以管理和清理，因此推荐使用**外部构建（Out-of-Source Build）**。

### 🚀 外部构建简介：

以编译 wxGTK 静态库和动态库为例：

1. 解压源码后创建 `static` 和 `shared` 两个子目录。

2. 进入 `static`，执行：

   ```Shell
   ../configure --enable-static
   make
   ```

   会生成静态库。

3. 进入 `shared`，执行：

   ```Shell
   ../configure --enable-shared
   make
   ```

   会生成动态库。

这种方式将构建文件与源码隔离，便于管理。

---

### 🛠 CMake 外部构建流程：

1. 清理 `t1` 目录中除 `main.c` 和 `CMakeLists.txt` 外的文件（特别是 `CMakeCache.txt`）。

2. 在 `t1` 中或任意位置新建 `build` 目录。

3. 进入 `build`，执行：

   ```Shell
   cmake ..
   ```

   生成 Makefile 和中间文件。

4. 运行：

   ```Shell
   make
   ```

   编译输出目标程序 `hello`，且所有中间文件仅存在于 `build` 目录中。

> ✅ **优点**：原始工程目录保持干净，构建过程可控、易清理。

---

### 📌 小贴士：

- `HELLO_SOURCE_DIR`：源代码路径，如 `/backup/cmake/t1`

- `HELLO_BINARY_DIR`：构建路径，如 `/backup/cmake/t1/build`



## 9.小结

本小节介绍了使用 CMake 构建 Hello World 程序的完整流程，涵盖了四个基本指令：`PROJECT`、`SET`、`MESSAGE`、`ADD_EXECUTABLE`，以及变量调用的方法。同时提到了两个隐式变量：`<projectname>_SOURCE_DIR` 和 `<projectname>_BINARY_DIR`。

尽管对于简单项目，直接用 `gcc main.c` 或手写 Makefile 更快捷，但当面对如 **kdelibs（压缩包高达 50M）** 这类大型工程时，使用 CMake 无疑更加高效和可维护。



下一节，我们的任务是让 Hello World 看起来更像一个工程。



# 四、更好一点的Hello World

从本小节开始，后面所有的构建我们都将采用 out-of-source 外部构建，约定的构建目录是工程目录下的 build 自录。

本小节的任务是让前面的 Hello World 更像一个工程，我们需要做的是：

1. 为工程添加一个`子目录 src`，用来放置工程源代码

2. 添加一个`子目录 doc`，用来放置这个工程的文档 `hello.txt`

3. 在工程目录添加文本文件 `COPYRIGHT`，`README`

4. 在工程目录添加一个 `runhello.sh` 脚本，用来调用 hello 二进制

5. 将构建后的目标文件放入构建目录的 `bin 子目录`

6. 最终安装这些文件：将 `hello` 二进制与 `runhello.sh` 安装至`/usr/bin`，将`doc 目录`的内容以及 `COPYRIGHT/README` 安装到`/usr/share/doc/cmake/t2`

## 1.准备工作

在`/backup/cmake/`目录下建立 `t2 目录`。

将 t1 工程的 `main.c` 和 `CMakeLists.txt` 拷贝到 t2 目录中。

## 2.添加子目录src

```Shell
mkdir src
mv main.c src
```

现在的工程看起来是这个样子：

一个子目录 src，一个 CMakeLists.txt。

上一节我们提到，需要为任何子目录建立一个 CMakeLists.txt，进入子目录 src，编写 CMakeLists.txt 如下：

```Shell
ADD_EXECUTABLE(hello main.c)
```

将 t2 工程的 CMakeLists.txt 修改为：

```Shell
PROJECT(HELLO C)
ADD_SUBDIRECTORY(src bin)
```

然后建立 build 目录，进入 build 目录进行外部编译：

```Shell
cmake ..
make
```

构建完成后，你会发现生成的目标文件 hello 位于 build/bin 目录中。

**语法解释：**

**ADD_SUBDIRECTORY 指令**

`ADD_SUBDIRECTORY(source_dir [binary_dir] [EXCLUDE_FROM_ALL])`

这个指令用于向当前工程添加存放源文件的子目录，并可以指定中间二进制和目标二进制存放的位置。

`EXCLUDE_FROM_ALL` 参数的含义是将这个目录从编译过程中排除，比如，工程的 example，可能就需要工程构建完成后，再进入 example 目录单独进行构建(当然，你也可以通过定义依赖来解决此类问题)。

上面的例子定义了将 `src 子目录`加入工程，并指定编译输出(包含编译中间结果)路径为 `bin 目录`。如果不进行 `bin 目录`的指定，那么编译结果(包括中间结果)都将存放在 `build/src 目录`(这个目录跟原有的 src 目录对应)，指定 `bin 目录`后，相当于在编译时将 src 重命名为 bin，所有的中间结果和目标二进制都将存放在 `bin 目录`。



这里需要提一下的是 `SUBDIRS 指令`，使用方法是：

`SUBDIRS(dir1 dir2...)`，但是这个指令已经不推荐使用。它可以一次添加多个子目录，并且，即使外部编译，子目录体系仍然会被保存。

如果我们在上面的例子中将 `ADD_SUBDIRECTORY (src bin)` 修改为 `SUBDIRS(src)`，那么在 build 目录中将出现一个 src 目录，生成的目标代码 hello 将存放在 src 目录中。



更推荐使用 `ADD_SUBDIRECTORY`，因其功能更灵活、现代。



## 3.换个地方保存目标二进制

不论是 SUBDIRS 还是 ADD_SUBDIRECTORY 指令(不论是否指定编译输出目录)，我们都可以通过 SET 指令重新定义 `EXECUTABLE_OUTPUT_PATH` 和 `LIBRARY_OUTPUT_PATH` 变量来指定最终的目标二进制的位置（指最终生成的 hello 或者最终的共享库，不包含编译生成的中间文件）。

```Shell
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
```

含义如下：

- `EXECUTABLE_OUTPUT_PATH`：设置最终生成的可执行文件（如 `hello`）输出到 `build/bin`。

- `LIBRARY_OUTPUT_PATH`：设置最终生成的库文件输出到 `build/lib`。



在第一节我们提到了 `<projectname>_BINARY_DIR` 和 `PROJECT_BINARY_DIR` 变量，它们指的是编译发生的当前目录，如果是内部编译，就相当于 `PROJECT_SOURCE_DIR` 也就是工程代码所在目录，如果是外部编译，指的是外部编译所在目录，也就是本例中的 `build 目录`。

所以，上面两个指令分别定义了：

可执行二进制的输出路径为 `build/bin` 和库的输出路径为 `build/lib`。

本节我们没有提到共享库和静态库的构建，所以，你可以不考虑第二条指令。

问题是，我应该把这两条指令写在`工程的 CMakeLists.txt` 还是 `src 目录下的CMakeLists.txt`，把握一个简单的原则，在哪里 `ADD_EXECUTABLE` 或 `ADD_LIBRARY`，如果需要改变目标存放路径，就在哪里加入上述的定义。

在这个例子里，当然就是指 `src 下的 CMakeLists.txt` 了。



## 4.如何安装

### 4.1 CMake 的安装机制简明概览

CMake 安装支持两种使用场景：

1. **直接安装**：通过 `make install` 将程序安装到系统目录（如 `/usr/bin`）。

2. **打包安装**：通过设置 `DESTDIR` 将安装内容临时放入指定路径，用于打包。



**手写 Makefile 示例**

```MakeFile
DESTDIR=
install:
    mkdir -p $(DESTDIR)/usr/bin
    install -m 755 hello $(DESTDIR)/usr/bin
```

说明：

- `make install`：安装到 `/usr/bin`

- `make install DESTDIR=/tmp/test`：安装到 `/tmp/test/usr/bin`，用于打包

若再引入 `PREFIX`，则：

```MakeFile
DESTDIR=
PREFIX=/usr
install:
    mkdir -p $(DESTDIR)/$(PREFIX)/bin
    install -m 755 hello $(DESTDIR)/$(PREFIX)/bin
```

这正是 autotools 类工具中 `./configure --prefix=/usr` 的作用。



**CMake 安装对应方式**

CMake 中提供了两大核心机制：

1. `CMAKE_INSTALL_PREFIX`：相当于 `PREFIX`

2. `DESTDIR`：临时根目录前缀，常用于打包

配置命令：

```Shell
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
```

执行安装：

```Shell
make install                 # 安装到 /usr/bin 等
make install DESTDIR=/tmp/p # 实际安装到 /tmp/p/usr/bin
```

`CMAKE_INSTALL_PREFIX` 变量 等价于传统 `./configure --prefix=...`



### 4.2 INSTALL 指令详解

1. **安装目标文件（可执行程序、库）**

   ```CMake
   INSTALL(TARGETS target1 [target2 ...]
           [RUNTIME DESTINATION <dir>]    # 可执行文件
           [LIBRARY DESTINATION <dir>]    # 动态库
           [ARCHIVE DESTINATION <dir>])   # 静态库
   ```

   - `TARGETS`：由 `add_executable` 或 `add_library` 生成的目标

   - `DESTINATION`：为相对路径时，会附加在 `CMAKE_INSTALL_PREFIX` 后

   - 自动处理 Debug/Release 等构建模式的库路径

   > 不用关心目标生成路径，直接指定目标名即可

   

   ✅ **示例：**

   ```CMake
   INSTALL(TARGETS myrun mylib mystaticlib
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION libstatic)
   ```

   安装后路径为：

   - 可执行文件：`${CMAKE_INSTALL_PREFIX}/bin/myrun`

   - 动态库：`${CMAKE_INSTALL_PREFIX}/lib/mylib.so`

   - 静态库：`${CMAKE_INSTALL_PREFIX}/libstatic/libmystaticlib.a`

   ---

1. **安装普通文件（如配置、文档等）**

   ```CMake
   INSTALL(FILES file1 file2 ...
           DESTINATION <dir>
           [PERMISSIONS ...]
           [RENAME <newname>]
           [OPTIONAL])
   ```

   - 默认权限为 `644`（OWNER_WRITE, OWNER_READ, GROUP_READ, WORLD_READ）

   - 可通过 `PERMISSIONS` 自定义权限

   ✅ 示例：

   ```CMake
   INSTALL(FILES config.ini README.md DESTINATION etc/myproj)
   
   ```

   ---

1. **安装可执行脚本（非目标）**

   ```CMake
   INSTALL(PROGRAMS script1.sh ...
           DESTINATION <dir>
           [PERMISSIONS ...]
           [RENAME <newname>]
           [OPTIONAL])
   ```

   - 用于安装非构建生成的脚本、可执行程序

   - 默认权限为 `755`（OWNER_EXECUTE 等）

   ✅ 示例：

   ```CMake
   INSTALL(PROGRAMS run.sh DESTINATION bin)
   ```

1. **安装目录及其内容**

   ```CMake
   INSTALL(PROGRAMS script1.sh ...
           DESTINATION <dir>
           [PERMISSIONS ...]
           [RENAME <newname>]
           [OPTIONAL])
   ```

   - `DIRECTORY` 后面的路径 **若无斜杠**，目录本身会被复制

   - 加 `/` 则只复制目录内容

   - `PATTERN`：匹配过滤文件/目录，可配合 `EXCLUDE` 或 `PERMISSIONS` 使用

   ✅ 示例：

   ```CMake
   INSTALL(DIRECTORY icons scripts/ DESTINATION share/myproj
           PATTERN "CVS" EXCLUDE
           PATTERN "scripts/*"
           PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ)
   ```

   此例说明：

   - `icons` 目录整体复制到 `share/myproj/icons`

   - `scripts/` 中的内容复制到 `share/myproj`（目录本身不复制）

   - 忽略名为 `CVS` 的文件/目录

   - 对 `scripts/*` 设置指定权限

   ---

1. **安装时执行脚本或代码**

```CMake
INSTALL(SCRIPT some_script.cmake)
INSTALL(CODE "MESSAGE(\"Installing something...\")")
```

- `SCRIPT` 执行外部 `.cmake` 文件

- `CODE` 直接执行 CMake 命令（字符串）



## 5.修改Helloworld支持安装

在本节开头我们定义了本节的任务如下：

1. 为工程添加一个`子目录 src`，用来放置工程源代码

2. 添加一个`子目录 doc`，用来放置这个工程的文档 `hello.txt`

3. 在工程目录添加文本文件 `COPYRIGHT`，`README`

4. 在工程目录添加一个 `runhello.sh` 脚本，用来调用 hello 二进制

5. 将构建后的目标文件放入构建目录的 `bin 子目录`

6. 最终安装这些文件：将 `hello` 二进制与 `runhello.sh` 安装至`/<prefix>/bin`，将`doc 目录`的内容以及 `COPYRIGHT/README` 安装到`/<prefix>/share/doc/cmake/t2`

首先我们先补上未添加的文件：



### 5.1 添加 `doc 目录`及文件

```Shell
cd /backup/cmake/t2
mkdir doc
vi doc/hello.txt
```

`hello.txt`随便填写一些内容并保存

### 5.2 在工程目录添加 `runhello.sh` 脚本

内容为：

```Shell
./hello
```

### 5.3 添加工程目录中的 `COPYRIGHT` 和 `README`

```Shell
touch COPYRIGHT
touch README
```



### 5.4 下面改写各目录的 CMakeLists.txt

1. 安装 COPYRIGHT/README，直接修改主工程文件 CMakelists.txt，加入以下指令：

   ```CMake
   INSTALL(FILES COPYRIGHT README DESTINATION share/doc/cmake/t2)
   ```

1. 安装 runhello.sh，直接修改主工程文件 CMakeLists.txt，加入如下指令：

   ```CMake
   INSTALL(PROGRAMS runhello.sh DESTINATION bin)
   ```

1. 安装 doc 中的 hello.txt，这里有两种方式：

   一是通过在 doc 目录建立 CMakeLists.txt 并将 doc 目录通`ADD_SUBDIRECTORY` 加入工程来完成。

   另一种方法是直接在工程目录通过 `INSTALL(DIRECTORY )`来完成，前者比较简单，各位可以根据兴趣自己完成，我们来尝试后者，顺便演示以下 DIRECTORY 的安装。

   因为 hello.txt 要安装到 `/prefix/share/doc/cmake/t2`，所以我们不能直接安装整个 doc 目录，这里采用的方式是安装 doc 目录中的内容，也就是使用 `doc/`。

   在工程文件中添加：

   ```CMake
   INSTALL(DIRECTORY doc/ DESTINATION share/doc/cmake/t2)
   ```

1. 在 `src`目录的 `CMakeLists.txt`文件中添加如下内容，以安装 hello 到 `/<prefix>/bin`中：

   ```CMake
   INSTALL(TARGETS hello RUNTIME DESTINATION bin)
   ```



## 6.尝试我们修改的结果

现在进入 build 目录进行外部编译，注意使用 `CMAKE_INSTALL_PREFIX` 参数，这里我们将它安装到了`/tmp/t2`目录：

```Shell
cmake -DCMAKE_INSTALL_PREFIX=/tmp/t2/usr ..
```

然后运行：

```Shell
make
make install
```

让我们进入/tmp/t2 目录看一下安装结果：

```Shell
./usr
./usr/share
./usr/share/doc
./usr/share/doc/cmake
./usr/share/doc/cmake/t2
./usr/share/doc/cmake/t2/hello.txt
./usr/share/doc/cmake/t2/README
./usr/share/doc/cmake/t2/COPYRIGHT
./usr/bin
./usr/bin/hello
./usr/bin/runhello.sh
```

如果你要直接安装到系统，可以使用如下指令：

`cmake -DCMAKE_INSTALL_PREFIX=/usr ..`

---

## 7.一个疑问

如果我没有定义 `CMAKE_INSTALL_PREFIX` 会安装到什么地方？

你可以尝试以下，`cmake ..;make;make install`，你会发现 `CMAKE_INSTALL_PREFIX` 的默认定义是 `/usr/local`。

---

## 8.小结

本小节主要描述了如何在工程中使用多目录、各种安装指令以及 CMAKE_INSTALL_PREFIX 变量(你真够牛的，这么点东西居然罗唆了这么多文字)。

在下一小节，我们将探讨如何在 cmake 中构建动态库和静态库，以及如何使用外部头文件和外部共享库，毕竟，这是程序编写中最长使用的（对了，你知道用怎样的 gcc 参数可以直接构建静态库和动态库吗？）



# 五、静态库和动态库的构建

读者云，太能罗唆了，一个 Hello World 就折腾了两个大节。OK，从本节开始，我们不再折腾 Hello World 了，我们来折腾 Hello World 的共享库。

本节的任务：

1. 建立一个静态库和动态库，提供 `HelloFunc 函数`供其他程序编程使用，HelloFunc 向终端输出 Hello World 字符串。

2. 安装头文件与共享库。

## 1.准备工作

在 `/backup/cmake` 目录建立 `t3 目录`，用于存放本节涉及到的工程。

## 2.建立共享库

```Shell
cd /backup/cmake/t3
mkdir lib
```

在 t3 目录下建立 `CMakeLists.txt`，内容如下：

```CMake
PROJECT(HELLOLIB)
ADD_SUBDIRECTORY(lib)
```

在 lib 目录下建立两个源文件 `hello.c` 与 `hello.h`

`hello.c` 内容如下：

```Shell
#include "hello.h"

void HelloFunc() 
{
    printf("Hello World\n");
}
```

`hello.h` 内容如下：

```Shell
#ifndef HELLO_H
#define HELLO_H
#include <stdio.h>
void HelloFunc();
#endif
```

在 lib 目录下建立 CMakeLists.txt，内容如下：

```Shell
SET(LIBHELLO_SRC hello.c)
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
```

## 3.编译共享库

仍然采用 out-of-source 编译的方式，按照习惯，我们建立一个 build 目录，在 build 目录中：

```Shell
cmake ..
make
```

这时，你就可以在 lib 目录得到一个 `libhello.so`，这就是我们期望的共享库。

> 在Windows的Cygwin环境下，该共享库为 `cyghello.dll`。

如果你要指定 `libhello.so` 生成的位置，可以通过在主工程文件 CMakeLists.txt 中修改 `ADD_SUBDIRECTORY(lib)` 指令来指定一个编译输出位置或者在 `lib/CMakeLists.txt` 中添加 `SET(LIBRARY_OUTPUT_PATH <路径>)` 来指定一个新的位置。



这两者的区别我们上一节已经提到了，所以，这里不再赘述，下面，我们解释一下一个新的指令 `ADD_LIBRARY` ：

```Shell
ADD_LIBRARY(libname [SHARED|STATIC|MODULE]
 [EXCLUDE_FROM_ALL]
 source1 source2 ... sourceN
)
```

你不需要写全 `libhello.so`，只需要填写 `hello` 即可，cmake 系统会自动为你生成 `libhello.X`。

类型有三种：

1. SHARED，动态库

2. STATIC，静态库

3. MODULE，在使用 dyld 的系统有效，如果不支持 dyld，则被当作 SHARED 对待。

`EXCLUDE_FROM_ALL` 参数的意思是这个库不会被默认构建，除非有其他的组件依赖或者手工构建。



## 4. 添加静态库

同样使用上面的指令，我们在支持动态库的基础上再为工程添加一个静态库，按照一般的习惯，静态库名字跟动态库名字应该是一致的，只不过后缀是.a 罢了。

下面我们用这个指令再来添加静态库：

`ADD_LIBRARY(hello STATIC ${LIBHELLO_SRC})`

然后再在 build 目录进行外部编译，我们会发现，静态库根本没有被构建，仍然只生成了一个动态库。因为 hello 作为一个 target 是不能重名的，所以，静态库构建指令无效。

如果我们把上面的 `hello` 修改为 `hello_static`：

`ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})`

就可以构建一个 `libhello_static.a` 的静态库了。

这种结果显示不是我们想要的，我们需要的是名字相同的静态库和动态库，因为 target 名称是唯一的，所以，我们肯定不能通过 ADD_LIBRARY 指令来实现了。这时候我们需要用到另外一个指令：

`SET_TARGET_PROPERTIES`，其基本语法是：

```Shell
SET_TARGET_PROPERTIES(target1 target2 ...
 PROPERTIES prop1 value1
 prop2 value2 ...
)
```

这条指令可以用来设置输出的名称，对于动态库，还可以用来指定动态库版本和 API 版本。

在本例中，我们需要作的是向 `lib/CMakeLists.txt` 中添加一条：

`SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")`

这样，我们就可以同时得到 `libhello.so/libhello.a` 两个库了。



与它对应的指令是：

`GET_TARGET_PROPERTY(VAR target property)`

具体用法如下例，我们向 `lib/CMakeListst.txt` 中添加：

```Shell
GET_TARGET_PROPERTY(OUTPUT_VALUE hello_static OUTPUT_NAME)
MESSAGE(STATUS “This is the hello_static OUTPUT_NAME:” ${OUTPUT_VALUE})
```

如果没有这个属性定义，则返回 NOTFOUND。



让我们来检查一下最终的构建结果，我们发现，`libhello.a` 已经构建完成，位于 `build/lib` 目录中，但是 `libhello.so` 却消失了。这个问题的原因是：cmake 在构建一个新的 target 时，会尝试清理掉其他使用这个名字的库，因此，在构建 `libhello.a` 时，就会清理掉 `libhello.so`。



为了回避这个问题，比如再次使用 `SET_TARGET_PROPERTIES` 定义 `CLEAN_DIRECT_OUTPUT` 属性。

向 `lib/CMakeLists.txt` 中添加：

```Shell
SET_TARGET_PROPERTIES(hello PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(hello_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
```

这时候，我们再次进行构建，会发现 `build/lib` 目录中同时生成了 `libhello.so` 和 `libhello.a`。



## 5. 动态库版本号

按照规则，动态库是应该包含一个版本号的，我们可以看一下系统的动态库，一般情况是：

libhello.so.1.2

libhello.so ->libhello.so.1

libhello.so.1->libhello.so.1.2

为了实现动态库版本号，我们仍然需要使用 `SET_TARGET_PROPERTIES` 指令。

具体使用方法如下：

`SET_TARGET_PROPERTIES(hello PROPERTIES VERSION 1.2 SOVERSION 1)`

VERSION 指代动态库版本，SOVERSION 指代 API 版本。

将上述指令加入 `lib/CMakeLists.txt` 中，重新构建看看结果。

在 build/lib 目录会生成：

libhello.so.1.2

libhello.so.1->libhello.so.1.2

libhello.so ->libhello.so.1



## 6. 安装共享库和头文件

以上面的例子，我们需要将 `libhello.a`, `libhello.so.x` 以及 `hello.h` 安装到系统目录，才能真正让其他人开发使用，在本例中我们将 hello 的共享库安装到 `/lib` 目录，将 `hello.h` 安装到 `/include/hello` 目录。

利用上一节了解到的 `INSTALL 指令`，我们向 `lib/CMakeLists.txt` 中添加如下指令：

```Shell
INSTALL(TARGETS hello hello_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
INSTALL(FILES hello.h DESTINATION include/hello)
```

注意，静态库要使用 ARCHIVE 关键字。

通过：

```Shell
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
make install
```

我们就可以将头文件和共享库安装到系统目录 `/usr/lib` 和 `/usr/include/hello` 中了。

在Mac中，请安装到 `/usr/local/lib目录下`。



## 7.小结

本小节，我们谈到了：

- 如何通过 `ADD_LIBRARY` 指令构建动态库和静态库。

- 如何通过 `SET_TARGET_PROPERTIES` 同时构建同名的动态库和静态库。

- 如何通过 `SET_TARGET_PROPERTIES` 控制动态库版本。

- 最终使用上一节谈到的 `INSTALL 指令`来安装头文件和动态、静态库。

在下一节，我们需要编写另一个高级一点的 Hello World 来演示怎么使用我们已经构建的构建的共享库 libhello 和外部头文件。



# 六、如何使用外部共享库和

抱歉，本节仍然继续折腾 Hello World。

上一节我们已经完成了 libhello 动态库的构建以及安装，本节我们的任务很简单：

- 编写一个程序使用我们上一节构建的共享库。

## 1. 准备工作

请在 `/backup/cmake` 目录建立 `t4 目录`，本节所有资源将存储在 t4 目录。

## 2.重复以前的步骤，建立src目录，编写源文件main.c，内容如下：

```C
#include <hello.h>

int main(void)
{
    HelloFunc();
    
    return 0;
}
```

编写工程主文件 `CMakeLists.txt`：

```C
PROJECT(HELLO)
ADD_SUBDIRECTORY(src)
```

编写 `src/CMakeLists.txt`：

```C
ADD_EXECUTABLE(main main.c)
```

---

## 3.外部构建

按照习惯，仍然建立 `build 目录`，使用 `cmake ..` 方式构建。

过程：

```C
cmake ..
make
```

构建失败，如果需要查看细节，可以使用第一节提到的方法：`make VERBOSE=1` 来构建。

错误输出为是：

```C
/backup/cmake/t4/src/main.c:1:19: error: hello.h: 没有那个文件或目录
```

## 4.引入头文件搜索路径

有时候我们在编写 C/C++ 工程时，使用的库（比如 `hello` 库）头文件并不位于系统默认的标准路径（如 `/usr/include`），而是放在了某个自定义路径下，比如

`/usr/include/hello/hello.h`

此时，编译器就无法直接找到该头文件。

**❌ 错误示范：**

很多新手第一反应是直接这样写：

```CMake
#include <hello.h>
```

然后编译器报错：`fatal error: hello.h: No such file or directory`。如果你不告诉编译器这个头文件在哪里，它当然找不到。



**✅ 正确做法：使用 `INCLUDE_DIRECTORIES` 告诉 CMake 头文件位置**

我们可以在 `CMakeLists.txt` 中使用 `INCLUDE_DIRECTORIES` 命令，显式添加头文件搜索路径。例如，在 `src/CMakeLists.txt` 中添加：

```CMake
include_directories(/usr/include/hello)
```

或者更现代的推荐方式是使用 **`target_include_directories`**：

```CMake
add_executable(demo main.c)
target_include_directories(demo PRIVATE /usr/include/hello)
```

这告诉编译器去 `/usr/include/hello` 目录下找头文件。之后，我们就可以在代码中正常使用：

```CMake
#include <hello.h>
```

现在，头文件找到了，但问题还没完！

如果你尝试编译：

```Shell
gcc main.c -o demo
```

可能会出现如下链接错误：

```CMake
main.c:(.text+0x12): undefined reference to `HelloFunc'
```

这表明虽然头文件被找到了，但编译器在链接阶段找不到 `HelloFunc` 函数的定义——它应该是在 `libhello.so` 或 `libhello.a` 中。



## 5.为target添加共享库

我们这一步的目标是让可执行程序 `main` 能用上我们之前写的共享库 `libhello`。为此，需要用到两个新的 CMake 指令：`LINK_DIRECTORIES` 和 `TARGET_LINK_LIBRARIES`。

### 5.1 `LINK_DIRECTORIES` —— 指定库文件在哪

**用法：**

```CMake
LINK_DIRECTORIES(路径1 路径2 ...)
```

这个指令的作用是告诉编译器“去这些路径下找库文件”。
通常只有在库文件不在系统默认路径（比如 `/usr/lib`）下时才需要用它。

> 本例中我们用不到它，可以先不用管。 

---

### 5.2`TARGET_LINK_LIBRARIES` —— 把库“挂”到目标程序上

**用法：**

```CMake
TARGET_LINK_LIBRARIES(目标程序 库名1 [库名2 ...])
```

这个指令就是把某个库链接到你要编译的程序或库上。

比如我们要把 `libhello` 加到 `main` 上，只需要在 `src/CMakeLists.txt` 里加上一行：

```CMake
TARGET_LINK_LIBRARIES(main hello)
```

也可以写得更明确一些：

```CMake
TARGET_LINK_LIBRARIES(main libhello.so)
```

这里的 `hello` 对应的是我们之前生成的共享库 `libhello.so`（系统在链接时会自动补上前缀 `lib` 和后缀 `.so`）。

---

### 5.3 编译并运行

回到 `build` 目录，重新构建项目：

```Shell
cmake ..
make
```

如果一切正常，你会在 `build/src` 目录下看到一个可执行文件 `main`。运行它：

```Shell
./src/main
```

输出应该是：

```CMake
Hello World
```

说明它已经成功调用了 `libhello` 提供的函数。

---

### 5.4 检查一下链接情况

以用 `ldd` 命令看看 `main` 实际链接了哪些库：

```Shell
ldd src/main
```

输出可能长这样：

```Shell
linux-gate.so.1 =>  (0xb7ee7000)
libhello.so.1 => /usr/lib/libhello.so.1 (0xb7ece000)
libc.so.6 => /lib/libc.so.6 (0xb7d77000)
/lib/ld-linux.so.2 (0xb7ee8000)
```

能看到 `libhello.so.1`，说明 `main` 已经和我们写的共享库连上了。

---

### 5.5 如果你想链接静态库呢？

也很简单，只要把链接指令改成：

```CMake
TARGET_LINK_LIBRARIES(main libhello.a)
```

然后重新构建项目。

再次运行 `ldd src/main`，这次你会发现：

```Shell
linux-gate.so.1 =>  (0xb7fa8000)
libc.so.6 => /lib/libc.so.6 (0xb7e3a000)
/lib/ld-linux.so.2 (0xb7fa9000)
```

`libhello.so` 不见了！说明程序这次是把 `libhello.a` 静态编译进去了，不再依赖外部共享库。



### 总结：

- `TARGET_LINK_LIBRARIES(main hello)` 就是把 `libhello` 挂到 `main` 上。

- 默认链接的是动态库（`.so`），想用静态库（`.a`）就写上全名。

- 用 `ldd` 可以验证程序到底链接了哪些库。

通过这些步骤，我们就成功地让 `main` 使用了 `libhello` 提供的功能啦！

---



## 6.特殊的环境变量：`CMAKE_INCLUDE_PATH` 和 `CMAKE_LIBRARY_PATH`

在使用 CMake 时，如果你的头文件或库文件**不在标准路径**（如 `/usr/include` 或 `/usr/lib`）下，就可能导致找不到文件、编译失败的问题。为了应对这种情况，CMake 提供了两个**环境变量**来扩展搜索路径：

- `CMAKE_INCLUDE_PATH`：用于额外查找头文件的路径

- `CMAKE_LIBRARY_PATH`：用于额外查找库文件的路径

> 注意：它们是**环境变量**，不是 CMake 脚本里的变量。



### 6.1 如何设置这些变量？

在使用 `cmake ..` 前，你可以通过以下方式在终端中设置：

```Shell
export CMAKE_INCLUDE_PATH=/usr/include/hello
export CMAKE_LIBRARY_PATH=/usr/lib/mylibs
cmake ..
```

或者用一行命令快速指定：

```Shell
CMAKE_INCLUDE_PATH=/usr/include/hello cmake ..
```

设置后，CMake 就会在你指定的路径中查找头文件或库。

---

### 6.2这些变量的作用场景

这两个变量的作用是配合 CMake 的 `FIND_` 系列指令使用，比如 `FIND_PATH()` 查找头文件、`FIND_LIBRARY()` 查找库文件。

举个例子，我们有一个头文件 `hello.h`，它存放在 `/usr/include/hello/` 中。常规做法是写死路径：

```CMake
INCLUDE_DIRECTORIES(/usr/include/hello)
```

但更通用、更灵活的写法是这样：

```CMake
FIND_PATH(myHeader hello.h)
IF(myHeader)
    INCLUDE_DIRECTORIES(${myHeader})
ENDIF()
```

这样写之后，如果你设置了：

```Shell
export CMAKE_INCLUDE_PATH=/usr/include/hello
```

CMake 会自动在指定路径中查找 `hello.h`，不需要你在 CMake 文件中写死具体位置。

同样的方式也适用于库文件查找，比如：

```CMake
FIND_LIBRARY(myLib hello)
```

搭配：

```Shell
export CMAKE_LIBRARY_PATH=/usr/lib/mylibs
```

CMake 就可以自动找到 `libhello.a` 或 `libhello.so`。

---

### 6.3需要特别注意的点

- **这两个环境变量不会直接影响编译器参数**，比如不会自动加上 `-I` 或 `-L`。

- **只有在使用 `FIND_PATH()` 或 `FIND_LIBRARY()` 这样的 CMake 查找指令时才会生效**。

- 因为 CMake 的很多模块都是通过 `FIND_` 指令实现的，所以这两个变量对整个 CMake 系统的可移植性非常重要。

### 小结

| 环境变量             | 作用                 | 使用方式                   |
| -------------------- | -------------------- | -------------------------- |
| `CMAKE_INCLUDE_PATH` | 查找头文件的额外路径 | 配合 `FIND_PATH()` 使用    |
| `CMAKE_LIBRARY_PATH` | 查找库文件的额外路径 | 配合 `FIND_LIBRARY()` 使用 |

通过使用这两个环境变量，你可以让项目在不同的环境中更加通用，不再依赖于硬编码的路径，同时也能更好地支持跨平台开发。

> [原文](https://flowus.cn/share/60438529-b7ad-457e-9ef4-3dc711a685da?code=E73CZF)
