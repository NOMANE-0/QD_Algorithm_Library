# Ceres

Ceres Solver 是一个开源 C++ 库，用于建模和解决大型复杂的优化问题。它是一个功能丰富、成熟且性能卓越的库，自 2010 年以来一直在 Google 的生产环境中使用。Ceres Solver 可以解决两种类型的问题。

- 具有边界约束的非线性最小二乘问题
- 一般无约束优化问题

## 安装

### apt安装

```bash
sudo apt install libceres-dev
```

### [编译安装](http://ceres-solver.org/installation.html)

- 安装依赖

```bash
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
```

- 拉取源码

```bash
git clone -b 2.2.0 https://github.com/ceres-solver/ceres-solver.git
```

- 编译安装

```bash
cd ceres-solver 
mkdir build && cd build
cmake ..
make     # 嫌慢的多线程编译 make -j<线程数>    
make install
```

- 测试

```bash
bin/simple_bundle_adjuster ../ceres-solver-2.2.0/data/problem-16-22106-pre.txt
```

输出结果应该是这样的

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  4.185660e+06    0.00e+00    1.09e+08   0.00e+00   0.00e+00  1.00e+04        0    2.18e-02    6.57e-02
   1  1.062590e+05    4.08e+06    8.99e+06   0.00e+00   9.82e-01  3.00e+04        1    5.07e-02    1.16e-01
   2  4.992817e+04    5.63e+04    8.32e+06   3.19e+02   6.52e-01  3.09e+04        1    4.75e-02    1.64e-01
   3  1.899774e+04    3.09e+04    1.60e+06   1.24e+02   9.77e-01  9.26e+04        1    4.74e-02    2.11e-01
   4  1.808729e+04    9.10e+02    3.97e+05   6.39e+01   9.51e-01  2.78e+05        1    4.75e-02    2.59e-01
   5  1.803399e+04    5.33e+01    1.48e+04   1.23e+01   9.99e-01  8.33e+05        1    4.74e-02    3.06e-01
   6  1.803390e+04    9.02e-02    6.35e+01   8.00e-01   1.00e+00  2.50e+06        1    4.76e-02    3.54e-01

Solver Summary (v 2.2.0-eigen-(3.4.0)-lapack-suitesparse-(7.1.0)-metis-(5.1.0)-acceleratesparse-eigensparse)

                                     Original                  Reduced
Parameter blocks                        22122                    22122
Parameters                              66462                    66462
Residual blocks                         83718                    83718
Residuals                              167436                   167436

Minimizer                        TRUST_REGION

Dense linear algebra library            EIGEN
Trust region strategy     LEVENBERG_MARQUARDT
                                        Given                     Used
Linear solver                     DENSE_SCHUR              DENSE_SCHUR
Threads                                     1                        1
Linear solver ordering              AUTOMATIC                 22106,16
Schur structure                         2,3,9                    2,3,9

Cost:
Initial                          4.185660e+06
Final                            1.803390e+04
Change                           4.167626e+06

Minimizer iterations                        7
Successful steps                            7
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                         0.043895

  Residual only evaluation           0.029855 (7)
  Jacobian & residual evaluation     0.120581 (7)
  Linear solver                      0.153665 (7)
Minimizer                            0.339275

Postprocessor                        0.000540
Total                                0.383710

Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 1.769759e-09 <= 1.000000e-06)
```

## CMakeLists.txt

```cmake
make_minimum_required(VERSION 3.5)

project(helloworld)

find_package(Ceres REQUIRED)

# helloworld
add_executable(helloworld helloworld.cc)
target_link_libraries(helloworld Ceres::ceres)
```

## 常见问题

### 定义ceres::Problem时会导致代码报错

修改cmakelist:链接ceres库时使用``CERES_LIBRARIES`而不是`CERES_LIBS`

```bash
add_executable(myCeres myCeres.cpp)
target_link_libraries(myCeres ${OpenCV_LIBS} ${CERES_LIBRARIES})
```
