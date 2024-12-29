// Copyright (c) [2024] [Initial Equationor]
// [circular harmonics] is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <vector>

// 谐波分量越多拟合效果越好，但最好不要超过处理器的最大并发数，每个分量可以并行计算然后求和，但数量过多的元素在并发求和中使用规约法会产生额外开销。
#define CH_MAX_BANDS 16
// There shouldn't be too many circular harmonic kernel to describe a single shape, if you want to simulate a really complex anyway, try to split it.
// 一个MultiCH对象里面不应该有太多CH，如果你一定要描述一个很复杂的形状，把它拆分成多个MultiCH。
#define CH_MAX_KERNEL 8

namespace CircHarm {
typedef float Lenth;
typedef float Radius;

struct Vector2d
{
    Lenth x;
    Lenth y;
};

struct CH {
    int nBands; // 谐波次数
    Lenth bias; // 直流分量
    Lenth weights[2*CH_MAX_BANDS]; // 谐波权重，必须是偶数
    Vector2d position; // 质心在父节点坐标系中的坐标
};

struct MultiCH {
    int nKernels;
    CH kernels[CH_MAX_KERNEL];
    Radius rotate;
    Vector2d position; // 质心在父节点坐标系中的坐标
};

Lenth CalculateR(CH& ch, Radius rad);
Vector2d CalculateNorm(CH& ch, Radius rad);
Lenth CalculateArea(CH& ch);

bool CDVector2d2CH(Vector2d p, CH ch);
std::vector<Vector2d> GetContour(MultiCH shape);
} // namespace CircHarm