# C++11 多线程

# 初识线程

## 简介

线程（Thread）和进程（Process）是操作系统中的两个基本概念，它们都用于执行程序代码，但它们之间存在一些关键的区别：

1. **定义**：
   1. **进程**：进程是操作系统进行资源分配和调度的一个独立单位。它是应用程序运行的实例，拥有独立的内存空间。每个进程至少有一个线程，即主线程。
   2. **线程**：线程是进程中的一个实体，是被系统独立调度和分派的基本单位。线程自身基本上不拥有系统资源，只拥有一点在运行中必不可少的资源（如执行栈），但它可以与同属一个进程的其他线程共享进程所拥有的全部资源

## 创建线程

C++11中使用多线程，需要包含`<thread>`库，使用`std::thread`来创建一个线程。  

```C++
#include <iostream>
#include <thread>

using namespace std;

int main() {
    std::thread th01;
    return 0;
}
```

## 等待线程

`join`等待该线程结束，在结束主线程。

```C++
#include <iostream>
#include <thread>

using namespace std;

void print_hello_world() {
    cout<<"Hello World"<<endl;
}

int main() {
    thread th01(print_hello_world);
    
    th01.join();
    
    return 0;
}
```

`detach` 可以将主程序和`th01`分离开，主线程结束，`th01`也会继续运行，不会报错。

```C++
int main() {
    thread th01(print_hello_world);
    
    th01.detach();
    
    return 0;
}
```

`joinable`先判断这个线程是否剖可以使用`join`再去使用，提高安全规范。

```C++
int main() {
    thread th01(print_hello_world);
    bool isJoin = th01.joinable();
    
    if(isJoin){
        th01.join();
    }
    
    return 0;
}
```

## 传递参数   

#### 传递形参

在 `th01` 创建时传入函数名后，继续传入该函数的参数，即可传入 `th01` 中

```C++
#include <iostream>
#include <thread>
#include <string>

using namespace std;

void print_hello_world(string msg) {
    cout<<msg<<endl;
}

int main() {
    thread th01(print_hello_world, "Hello World");
    
    th01.join();
    
    return 0;
}
```

#### 传递实参

想要传递实参，需要使用 `std::ref()`传入。

```C++
#include <iostream>
#include <thread>

using namespace std;

void foo(int &x) {
    x += 1;
}

int main() {
    int a = 1;
    thread th01(foo, ref(a));
    th01.join();
    
    cout<<"a = "<<a<<endl;
    
    return 0;
}
```

举一个编译不会报错，运行会报错的例子。

```C++
#include <iostream>
#include <thread>

using namespace std;

thread t;

void foo(int &x) {
    x += 1;
}

void test(){
    int a = 1;
    t = thread(foo, ref(a));
}

int main() {
   test();
   t.join();
    
    return 0;
}
```

由于`a`是`test`中的一个局部变量，`test`执行结束的时候`a`已经被操作系统释放了，在`t`调用`foo`函数时，无法传入变量`a`，导致报错。

#### 传递类

##### 使用类的对象作为参数

##### 传入副本

```C++
#include <iostream>
#include <thread>
#include <memory>

class A{
public:
    void doSomething() {
        std::cout << "Doing something" << std::endl;
    }
};

void threadFunction(A obj) {
    obj.doSomething();
}

int main() {
    A myObject;
    std::thread t(threadFunction, myObject);
    t.join();
    return 0;
}
```

##### 使用引用

```C++
void threadFunction(A & obj) {
    obj.doSomething();
}

int main() {
    A myObject;
    std::thread t(threadFunction, std::ref(myObject)); // 传递对象的引用
    t.join();
    return 0;
}
```

##### 使用智能指针

```C++
void threadFunction(std::shared_ptr<A> obj) {
    obj->doSomething();
}

int main() {
    auto myObject = std::make_shared<A>();
    std::thread t(threadFunction, myObject);
    t.join();
    return 0;
}
```

#### 传递类内函数

当使用`std::thread`创建线程并传递类成员函数时，需要使用&来获取成员函数的地址，同时还需要传递对象的指针（或引用）作为第一个参数。

```C++
#include <iostream>
#include <thread>
#include <memory>

class A{
public:
    void doSomething() {
        std::cout << "Doing something" << std::endl;
    }
};

int main(){
    A myObject;
    std::thread t(&A::doSomething, &myObject);
    t.join();
    return 0;
}
```

推荐使用智能指针

```C++
int main() {
    auto myObject = std::make_shared<A>();
    std::thread t(&A::doSomething, myObject);
    t.join();
    return 0;
}
```

入口函数为类内私有成员 

 使用友元进行访问。

```C++
#include <iostream>
#include <thread>
#include <memory>

class A{
private:
    friend void thread_func();
    void doSomething() {
        std::cout << "Doing something" << std::endl;
    }
};

void thread_func(){
    auto myObject = std::make_shared<A>();
    std::thread t(&A::doSomething, myObject);
    t.join();
}

int main(){

    thread_func();
    
    return 0;
}
```

## 数据共享

在多个线程中共享数据时，需要注意线程安全问题。如果多个线程同时访问同一个变量，并且其中至少有一个线程对该变量进行了写操作，那么就会出现数据竞争问题。数据竞争可能会导致程序崩溃、产生未定义的结果，或者得到错误的结果。

数据竞争例子：`i` 初始值时0；线程A要对变量`i`进行`++`的操作，线程B也要对变量`i`进行`++`的操作；两个线程同时运行，那么将会出现数据竞争问题，线程B要对变量`i`进行`++`时，`i`正在被线程A调用进行`++`操作。

```C++
#include <iostream>
#include <thread>

using namespace std;

int a = 0;

void func(){
   for(int i = 0; i < 10000; i++){
       a ++;
   }
}

int main(){
    thread A(func);
    thread B(func);
    
    A.join();
    B.join();
    
    cout<<"a = "<<a<<endl;
    return 0;
}
```

预期是`a=20000`，但输出的结果并不是。

为了避免数据竞争问题，需要使用同步机制来确保多个线程之间对共享数据的访问是安全的。常见的同步机制包括互斥量、条件变量、原子操作等。

### 互斥量

下面使用简单的互斥量来解决上面的问题；

需要包含锁的头文件`<mutex>`

```C++
#include <iostream>
#include <thread>
#include <mutex>

using namespace std;

int a = 0;
mutex mtx;

void func(){
   for(int i = 0; i < 10000; i++){
       mtx.lock();
       a ++;
       mtx.unlock();
   }
}

int main(){
    thread A(func);
    thread B(func);
    
    A.join();
    B.join();
    
    cout<<"a = "<<a<<endl;
    return 0;
}
```

 这样，在线程A在执行`++`时，`a`加锁，线程B不可访问变量`a`，A在执行完`++`后，`a`解锁，线程B可访问变量`a`。

#### 互斥量死锁

假设有两个线程 T1 和 T2，它们需要对两个互斥量 mtx1 和 mtx2 进行访问，而且需要按照以下顺序获取互斥量的所有权:

- T1 先获取 mtx1 的所有权，再获取 mtx2 的所有权。
- T2 先获取 mtx2 的所有权，再获取 mtx1 的所有权。

如果两个线程同时执行，就会出现死锁问题。因为T1 获取了 mtx1 的所有权，但是无法获取 mtx2 的所有权，而 T2 获取了 mtx2 的所有权，但是无法获取 mtx1 的所有权，两个线程互相等待对方释放互斥量，导致死锁。

代码演示：

```C++
#include <iostream>
#include <thread>
#include <mutex>
#include <windows.h> // windows系统使用的Sleep

using namespace std;

mutex mtx1;
mutex mtx2;

void func1() {
    mtx1.lock();
    mtx2.lock();
    Sleep(2000);
    mtx2.unlock();
    mtx1.unlock();
    
}

void func2() {
    mtx2.lock();
    mtx1.lock();
    Sleep(2000);
    mtx1.unlock();
    mtx2.unlock();
}

int main() {
    thread A(func1);
    thread B(func2);

    A.join();
    B.join();
    
    cout << "over" << endl;

    return 0;
}
```

解决方法：调换好互斥量的加锁和解锁顺序。

- T1 先获取 mtx1 的所有权，再获取 mtx2 的所有权。
- T2 先获取 mtx1 的所有权，再获取 mtx2 的所有权。

```C++
#include <iostream>
#include <thread>
#include <mutex>
#include <windows.h> // windows系统使用的Sleep

using namespace std;

mutex mtx1;
mutex mtx2;

void func1() {
    mtx1.lock();
    mtx2.lock();
    Sleep(2000);
    mtx1.unlock();
    mtx2.unlock();
    
}

void func2() {
    mtx1.lock();
    mtx2.lock();
    Sleep(2000);
    mtx1.unlock();
    mtx2.unlock();
}

int main() {
    thread A(func1);
    thread B(func2);

    A.join();
    B.join();
    
    cout << "over" << endl;

    return 0;
}
```

#### 互斥量模板

1. `std::``lock_gurad`能够在作用域内自动加锁解锁

自动加锁解锁原理：

- 模板类创造时，构造函数被调用，该互斥量会被自动锁定。
- 模板类释放时，析构函数被调用，该互斥量会被自动解锁。

特点：`std::``lock_gurad`对象不能复制或者移动，因此只能它只能在局域中使用。

普通的加锁解锁：

```C++
#include <iostream>
#include <thread>
#include <mutex>

int share_data = 0;
std::mutex mtx;

void func() {
    mtx.lock();
    share_data += 1;
    mtx.unlock();
}

int main() {
    std::thread A(func);
    std::thread B(func);
    A.join();
    B.join();
    
    std::cout << "over" << std::endl;
}
```

1. 使用`std::``lock_gurad`加锁解锁：

```C++
void func() {
    std::lock_gurad<std::mutex> lg(mtx);
    share_data += 1;
}
```

`std::``unique_lock`是C++ 标准库中提供的一个互斥量封装类，用于在多线程程序中对互斥量进行加锁和解锁操作。它的主要特点是可以对互斥量进行更加灵活的管理，包括延迟加锁、条件变量、超时等。

`std::``unique_lock`提供了以下几个成员函数:

- `1ock``()`:尝试对互斥量进行加锁操作，如果当前互斥量已经被其他线程持有，则当前线程会被阻塞，直到互斥量被成功加锁。
- `try_lock``()`:尝试对互斥量进行加锁操作，如果当前互斥量已经被其他线程持有，则函数立即返回 false，否则返回 true。
- `try_lock_for``(const std::chrono::duration<Rep, Period>& rel time)`试对互斥量进行加锁操作，如果当前互斥量已经被其他线程持有，则当前线程会被阻塞，直到互斥量被成功加锁，或者超过了指定的时间。
- `try_lock_until``(const std::chrono::time point<Clock, Duration>& abs time`尝试对互斥量进行加锁操作，如果当前互斥量已经被其他线程持有，则当前线程ne ):会被阻塞，直到互斥量被成功加锁，或者超过了指定的时间点。
- `unlock``()`:对互斥量进行解锁操作。

使用`std::``unique_lock`自动加锁解锁：

```C++
void func() {
    std::unique_lock<std::mutex> lg(mtx);
    share_data += 1;
}
```

构造时传入参数`std::defer_lock`时，将不会自动加锁（所以需要手动加锁）：

```C++
void func() {
    std::unique_lock<std::mutex> lg(mtx, std::defer_lock);
    lg.lock();
    share_data += 1;
}
```

但是手动加锁解锁那为什么还要用这个模板？因为后面有更多加锁的方法。

1. 使用`try_lock_for()`可以进行延迟等待加锁：

```C++
// 需要支持时间的互斥锁
std::timed_mutex time_mtx;
void func() {
    std::unique_lock<std::timed_mutex> lg(time_mtx, std::defer_lock);
    // 等待互斥量进行加锁，时间为3秒钟，如果3秒后互斥量还未加锁，直接retrun false，反之return true
    if(lg.try_lock_for(std::chrono::seconds(3))){
        share_data += 1;
    }
}
```

正常的`lock()`会一直等待`unlock()`后再执行，而`try_lock_for()`支持等待时间超过后结束等待

再举个例子，下面的运行完`share_data`结果会是3而不是4。

```C++
#include <iostream>
#include <thread>
#include <mutex>

using namespace std;

int share_data = 0;
mutex mtx;

// 需要支持时间的互斥锁
std::timed_mutex time_mtx;
void func() {
    for (int i = 0; i < 2; i++) {
        std::unique_lock<std::timed_mutex> lg(time_mtx, std::defer_lock);
        // 等待互斥量进行加锁，时间为3秒钟，如果3秒后互斥量还未加锁，直接retrun false，反之return true
        if (lg.try_lock_for(std::chrono::seconds(2))) {
            // 让线程等待2秒后再继续
            std::this_thread::sleep_for(std::chrono::seconds(2));
            share_data += 1;
            
        }else {
        std::cout << "Lock not acquired" << std::endl;
        // 未能在指定时间内获取互斥锁
    }
    }
}

int main() {
    thread A(func);
    thread B(func);

    A.join();
    B.join();

    cout << "share_data = " << share_data << endl;
    return 0;
}
```

因为B线程中`try_lock_for`在等待过程中，因为A线程需要等待1秒后继续，B线程`try_lock_for`在第一次循环的时候，等了2秒，还没等到A线程的解锁，此时B线程的`try_lock_for`返回`false`，跳过了第一次循环，后面A线程执行好两次`+=1`后，B线程才拿到解锁，但只能`+=1`一次。

1. `try_lock_until()`的使用：

`try_lock_until()`与`try_lock_for()`功能一样，`try_lock_until()`是绝对时间，`try_lock_for()`是相对时间

```C++
void func() {
    for (int i = 0; i < 2; i++) {
        std::unique_lock<std::timed_mutex> lg(time_mtx, std::defer_lock);
        // 设置超时时间点，例如从现在开始的1000毫秒后
        auto timeout = std::chrono::steady_clock::now() + 
        std::chrono::milliseconds(1000);
        
        if (lg.try_lock_until(timeout) {
            share_data += 1;
            
        } else {
        std::cout << "Lock not acquired" << std::endl;
        // 未能在指定时间内获取互斥锁
    }
    }
}
```

## 单例模式

单例模式是一种常用的软件设计模式，它保证一个类只有一个实例，并提供一个全局访问点。它可以帮助协调系统整体的行为，比如程序配置信息由一个单例对象统一读取，其他对象再通过这个单例对象获取这些配置信息，简化了复杂环境下的配置管理。

下面写一个日志类。

代码示例：

```C++
class Log {
public:
    // 默认构造函数
    Log() {};

    // 拷贝构造函数被删除，这意味着不能通过复制来创建Log对象
    Log(const Log& log) = delete;

    // 拷贝赋值运算符被删除，这意味着不能通过赋值来复制Log对象
    Log& operator=(const Log& log) = delete;
    
    static Log& GetInstance(){
        static Log log;
        return log;
    }
    
    void PrintLog(std::string msg){
        std::cout<<__TIME__<<' '<<msg<<std::endl;
    }
};

int main(){
    Log::GetInstance().PrintLog("error");
}
```

`std::``call_once`的使用：

## 多线程特性：

`std::cref` 与`std::ref`：

在C++中，`std::cref`（`std::cref`是C++11引入的）和`std::ref`都是用来创建引用包装器（reference wrapper）的函数模板，它们位于`<utility>`头文件中。引用包装器是一种将引用操作封装进对象的方式，这在需要传递参数时特别有用，尤其是当参数需要通过函数传递但又不想复制该参数时。

1. **std::cref (std::cref is short for "copy reference")**

1. `std::cref`用于创建一个对const引用的引用包装器。它通常用于通过拷贝传递一个引用的拷贝，使得这个引用可以被存储和赋值。使用`std::cref`时，传入的引用在`std::cref`对象的生命周期内必须有效。

   例如：

```C++
const int& cref = someInt;
auto refWrapper = std::cref(cref); // 创建一个对const引用的引用包装器
```

   在多线程环境中，如果被引用的对象不会在多个线程中被修改，使用`std::cref`可以安全地传递对数据的const引用。

1. **std::ref (std::ref is short for "reference")**

1. `std::ref`用于创建一个对非常量引用的引用包装器。它允许非const引用被传递给函数，并且这个引用可以被存储和赋值。使用`std::ref`时，也需要保证传入的引用在`std::ref`对象的生命周期内有效。

   例如：

```C++
int& ref = someInt;
auto refWrapper = std::ref(ref); // 创建一个对非常量引用的引用包装器
```

   `std::ref`常用于避免在函数调用中不必要地复制参数，尤其是对于大型对象或需要修改原对象的情况。

在提供的代码片段中：

```C++
std::cref(cloudOrigin), std::ref(point_index_list[i]), std::ref(ignored_list[i]),
          std::ref(outliers_list[i]), std::ref(map))
```

- `std::cref(cloudOrigin)`创建了一个对`cloudOrigin`的const引用的引用包装器。由于`cloudOrigin`在这里可能是按const引用传递的，所以使用`std::cref`确保了它在函数内部不会被修改。
- `std::ref(point_index_list[i])`、`std::ref(ignored_list[i])`、和`std::ref(outliers_list[i])`创建了对这些非const引用的引用包装器。这意味着这些列表对象可以在`GroundSegmentation::insert_cloud`函数内部被修改。
- `std::ref(map)`创建了一个对`map`的引用包装器。由于`map`是一个非const引用，它允许`GroundSegmentation::insert_cloud`函数修改`map`对象中的数据。

使用`std::cref`和`std::ref`的好处是它们允许按引用传递参数，从而避免了不必要的复制，同时还可以保持函数签名的简洁性。此外，它们在多线程环境中非常有用，因为它们允许线程安全地传递对数据的引用，只要确保引用有效且没有竞态条件。

### 多线程处理数据：

```Markdown
在这段代码中，`start` 和 `end` 用于确定点云数据中每个线程需要处理的点的范围。这是一种常见的方法，用于在多个线程之间均匀分配工作负载，通常称为数据分割（data partitioning）或工作分割（work partitioning）。

这里，`i` 是当前线程的索引，`threadcount` 是线程的总数，而 `cloud->points.size()` 是点云中点的总数。计算 `start` 和 `end` 的目的是为每个线程分配一个大致相同数量的点，以便进行处理。

### `start` 的作用：

`start` 是每个线程开始处理的点的索引。它是通过向下取整计算得出的，确保即使在点云大小不能被线程数整除的情况下，每个线程也有工作可做。

### `end` 的作用：

`end` 是每个线程结束处理的点的索引加一（因为 `end` 是不包括在内的）。它是通过向上取整计算得出的，确保所有的点都被分配给了线程，即使这意味着最后一个线程可能会处理更多的点。

### 示例：

假设我们有以下情况：

- 点云中的点总数为 100 (`cloud->points.size() = 100`)。
- 线程数为 4 (`threadcount = 4`)。

计算 `start` 和 `end`：

- 第一个线程（`i = 0`）：
  - `start = std::floor((0 * 100) / 4) = 0`
  - `end = std::ceil(((0 + 1) * 100) / 4) = std::ceil(25) = 25`
  
- 第二个线程（`i = 1`）：
  - `start = std::floor((1 * 100) / 4) = std::floor(25) = 25`
  - `end = std::ceil(((1 + 1) * 100) / 4) = std::ceil(50) = 50`

- 第三个线程（`i = 2`）：
  - `start = std::floor((2 * 100) / 4) = std::floor(50) = 50`
  - `end = std::ceil(((2 + 1) * 100) / 4) = std::ceil(75) = 75`

- 第四个线程（`i = 3`）：
  - `start = std::floor((3 * 100) / 4) = std::floor(75) = 75`
  - `end = std::ceil(((3 + 1) * 100) / 4) = std::ceil(100) = 100`

### 注意：

- 由于向上取整和向下取整的使用，最后一个线程可能会处理更多的点，以确保所有点都被处理。
- 如果点云的大小正好是线程数的倍数，那么每个线程处理的点数将完全相同。

通过这种方式，每个线程都被分配了一个点的子集，可以并行地处理这些点，从而提高整体的处理速度
```