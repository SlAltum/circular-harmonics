[English](doc/en/README.md)
# 先决条件
## 环境
c++17
## 使用
TODO: 使用opencl并行计算和eigen矩阵计算库
## Test && Example
1. SDL2
2. dear-imgui
3. gtest
# 如何开始
1. 把`imgui`头文件复制到`3rdparty/imgui/*`
2. 安装SDL2和gtest（需要有.cmake文件）
3. `cmake -B build` && `cmake --build build`
# 什么是圆谐函数
圆谐函数（circular harmonics function, CH）是正弦谐波函数在极坐标下的表现形式，具体形式如下：
```
ρ = b + w_0 * cosθ  + w_1 * sinθ
      + w_2 * cos2θ + w_3 * sin2θ
      + w_4 * cos3θ + w_5 * sin3θ
      + ...
      + w_{2n-2} * cos(nθ) + w_{2n-1} * sin(nθ)
```
其中ρ和θ是极径和极角，b是直流分量，w_0 - w_n是高次谐波分量。
调和函数（harmonics function）在物理学和信号处理中应用广泛，但本项目着眼于其几何特性并致力于使用CH构建二维物理模拟系统。
# 算法描述
[算法](doc/algorithm.md)