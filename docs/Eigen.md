# Eigen tf的使用与几何变换
## 初始化 Eigen 进行多线程操作:

```C++
Eigen::initParallel();
```

`Eigen::initParallel();` 是 Eigen 库中的一个函数调用，用于初始化 Eigen 进行多线程操作。Eigen 是一个高级 C++ 库，用于线性代数、矩阵和向量运算，它支持多种数值方法和算法。

在多核处理器上，多线程可以显著提高大规模矩阵运算的性能。Eigen 库提供了对多线程的支持，允许其算法在多个 CPU 核心上并行执行。然而，为了利用这种并行性，你需要在程序的某个地方调用 `Eigen::initParallel();`。

以下是 `Eigen::initParallel();` 的一些关键点：

1. **初始化****多线程****支持**：这个调用告诉 Eigen 库，你的应用程序打算使用多线程进行计算。

1. **设置 OpenMP**：Eigen 的多线程支持通常是通过 OpenMP 实现的，这是一个广泛使用的多平台并行编程模型。调用 `Eigen::initParallel();` 将初始化 OpenMP 环境。

1. **性能优化**：启用多线程可以提高大规模矩阵运算的速度，特别是当处理的数据集大小超出了单个CPU核心的处理能力时。

1. **调用时机**：`Eigen::initParallel();` 通常在 `main()` 函数或程序的早期阶段调用，以确保在进行任何并行计算之前完成多线程环境的初始化。

1. **环境变量**：Eigen 的多线程行为也可以通过设置环境变量来控制，例如 `OMP_NUM_THREADS` 可以设置使用的线程数。

1. **编译时配置**：为了启用多线程支持，你的应用程序需要链接到 Eigen 的多线程版本，并且在编译时可能需要指定额外的编译器标志，如 `-pthread` 或 `/openmp`（取决于你的编译器和平台）。

请注意，多线程可能会引入额外的复杂性，如线程安全和同步问题，因此在并行化关键代码段时需要小心。此外，对于小规模的计算，多线程可能不会带来性能上的提升，反而可能因为线程管理的开销而降低性能。通常建议对不同规模的问题进行性能测试，以确定多线程是否有益。

# 【Eigen】旋转方向及eulerAngles函数参数说明

1. 旋转方向

不同的几何库对于旋转方向的正负号问题的定义不尽相同。这里主要验证下`Eigen`库旋转时，正负号判定的问题。

### 1.1 绕X轴旋转一个正角度

```C++
void TEST_rotation_direction_positive_negative_view_from_x_positive_to_origin()
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
    Eigen::Vector3d input_point(0, 1, 0);
    Eigen::Vector3d output_point = R * input_point;

    std::cout << "output_point.y: " << output_point(1) << std::endl;
    std::cout << "output_point.z: " << output_point(2) << std::endl;
    if(output_point[2] > 0)
        std::cout << "逆时针为正" << std::endl;
    else
        std::cout << "顺时针为正" << std::endl;
}
```

输出如下：

> output_point.y: 0.707107 output_point.z: 0.707107 逆时针为正

### 1.2 绕Y轴旋转一个正角度

```C++
void TEST_rotation_direction_positive_negative_view_from_y_positive_to_origin()
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY());
    Eigen::Vector3d input_point(0, 0, 1);
    Eigen::Vector3d output_point = R * input_point;

    std::cout << "output_point.x: " << output_point(0) << std::endl;
    std::cout << "output_point.y: " << output_point(2) << std::endl;
    if(output_point[0] > 0)
        std::cout << "逆时针为正" << std::endl;
    else
        std::cout << "顺时针为正" << std::endl;
}
```

输出如下：

> output_point.x: 0.707107 output_point.y: 0.707107 逆时针为正

### 1.3 绕Z轴旋转一个正角度

```C++
void TEST_rotation_direction_positive_negative_view_from_z_positive_to_origin()
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d input_point(1.0, 0, 0);
    Eigen::Vector3d output_point = R * input_point;

    std::cout << "output_point.x: " << output_point(0) << std::endl;
    std::cout << "output_point.y: " << output_point(1) << std::endl;
    if(output_point[1] > 0)
        std::cout << "逆时针为正" << std::endl;
    else
        std::cout << "顺时针为正" << std::endl;
}
```

输出如下：

> output_point.x: 0.707107 output_point.y: 0.707107 逆时针为正

1.4 结果 由任意一个轴的正方向看向坐标系的原点，逆时针为正

## 2. eulerAngles函数参数说明

eulerAngles()用于将旋转矩阵转换为欧拉角，使用时需要根据实际情况指定绕轴旋转的顺序。eulerAngles()的函数原型如下： Eigen::vector3d eulerAngles(Index a0, Index a1, Index a2)，其中参数a0 a1 a2 用 0 1 2表示旋转轴，其中0表示X轴，1表示Y轴，2表示Z轴。此外，a0 表示首先选择的轴，a1 表示其次旋转的轴，a2 表示最后旋转的轴。输出结果的顺序与轴旋转的顺序相同。 以下通过两种情况进行验证：

### 2.1 先绕X轴旋转，其次绕Y轴旋转，最后绕Z轴旋转

```C++
void TEST_x30_y45_z60()
{
    double x = M_PI / 6;
    double y = M_PI / 4;
    double z = M_PI / 3;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());

    auto angle = R.eulerAngles(0, 1, 2) * 180 / M_PI;
    std::cout << "X  = " << angle(0) << std::endl;
    std::cout << "Y  = " << angle(1) << std::endl;
    std::cout << "Z  = " << angle(2) << std::endl;
    if ((std::abs(angle(0) - 30) < 0.001) &&
        (std::abs(angle(1) - 45) < 0.001) &&
        (std::abs(angle(2) - 60) < 0.001))
    {
        std::cout << "xyz" << std::endl;
    }
}
```

输出结果如下：

> X = 30 Y = 45 Z = 60 xyz

可以使用

```C++
Vector3f ea = R.eulerAngles(0, 1, 2);
```

![img](images/-17532561731671.png)