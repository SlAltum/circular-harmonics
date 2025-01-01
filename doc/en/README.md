# Prerequisite
## Environment
c++17
## Use
TODO: use opencl parallel calculation and eigen matrix library
## Test && Example
1. SDL2
2. dear-imgui
3. gtest
# How to Start
1. Copy `imgui` header files into `3rdparty/imgui/*`
2. install SDL2 and gtest
3. `cmake -B build` && `cmake --build build`
# What is Circular Harmonics Function
Circular harmonics(CH) function is cosine harmonic function in polar coordinate, common form show like:
```
ρ = b + w_0 * cosθ  + w_1 * sinθ
      + w_2 * cos2θ + w_3 * sin2θ
      + w_4 * cos3θ + w_5 * sin3θ
      + ...
      + w_{2n-2} * cos(nθ) + w_{2n-1} * sin(nθ)
```
where ρ and θ means radial and angular namely in polar coordinate, b means DC component and w_0 - w_n means weights of harmonics bands.

Despite that harmonics function often used in physics and signal processing, this project mainly focus on its geometric features and try to use CH to build a 2d phisic simulation system.
# Algorithms
[algorithm](algorithm.md)