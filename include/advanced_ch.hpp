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
#include <cstring>
#include <list>
#define CH_MAX_WEIGHTS 8

namespace AdvancedCH {
typedef float Lenth;
typedef float Radius;

struct Vector2d {
    Lenth x;
    Lenth y;
}; // struct Vector2d

struct Ray {
    Vector2d start;
    Vector2d direction;
};

class AdvancedCH {
public:
    int m_nWeights; // 谐波分量个数
    Lenth m_bias; // 直流分量
    int m_iWeights[CH_MAX_WEIGHTS]; // 谐波分量下标
    Lenth m_weights[CH_MAX_WEIGHTS]; // 谐波分量
    Vector2d m_position; // 质心位置
    Radius m_rotation = 0; // 旋转弧度
    virtual ~AdvancedCH() {};
    virtual AdvancedCH* Clone() const = 0;
    virtual void Reset();

    /// 计算圆谐函数在theta(+m_position)处的极径
    /// \param theta 相对于质心的逆时针（假设x-y是右手坐标系）转角
    /// \return 圆谐函数在theta+m_position处的极径
    Lenth GetRho(Radius theta);

    /// 计算圆谐函数在theta(+m_position)处的归一化法向量。
    /// TODO: 避免重复调用GetRho和DiffRho以优化性能
    /// \param theta 相对于质心的逆时针（假设x-y是右手坐标系）转角
    /// \return 笛卡尔直角坐标系下的归一化法向量（外法线方向）
    Ray GetNormal(Radius theta);

    /// 对另一个圆谐核进行碰撞检测
    /// \param another
    /// \return
    bool CollideAdvancedCH(AdvancedCH* another);
}; // class AdvancedCH

class KineticCH : public AdvancedCH {
protected:
    Lenth m_area; // 面积

public:
    Lenth m_mass; // 质量
    Lenth m_inertiaMult; // 转动惯量乘数 I=mass*inertia
    Vector2d m_velocity = { 0 }; // 线速度
    Radius m_omega = 0; // 角速度
    Lenth m_elastic = 1; // 碰撞恢复系数
    Lenth m_friction = 0.1; // 摩擦系数

    /// 计算面积和转动惯量
    void CalculateInertia();

    /// 获取函数包络在theta(+m_position)处的速度矢量
    /// \param theta 相对于质心的逆时针（假设x-y是右手坐标系）转角
    /// \return 速度矢量
    Vector2d GetSurfaceVelocity(Radius theta);

    /// 附带刚体运动模拟的碰撞检测
    /// \param another
    /// \return
    bool CollideKineticCH(KineticCH* another);

    /// 运动
    /// \param dTime 时间单元，单位秒
    void Move(float dTime);
}; // class KineticCH

} // namespace AdvancedCH