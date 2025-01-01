// Copyright (c) [2024] [Initial Equationor]
// [circular harmonics] is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
#include "advanced_ch.hpp"

namespace AdvancedCH {
void AdvancedCH::Reset() {
    m_nWeights = 0;
    m_bias = 0;
    memset(m_iWeights, 0, CH_MAX_WEIGHTS*sizeof(int));
    memset(m_weights, 0, CH_MAX_WEIGHTS*sizeof(Lenth));
    m_position = Vector2d{ 0 };
    m_rotation = 0;
} // void AdvancedCH::Reset

Lenth AdvancedCH::GetRho(Radius theta) {
    Radius _theta = theta + m_rotation;
    Lenth rho = m_bias;
    for (int i = 0; i < m_nWeights; ++i) {
        int iBand = m_iWeights[i] / 2;
        if (m_iWeights[i] % 2 == 0) {
            rho += m_weights[i] * cos((iBand+1)*(_theta));
        } else {
            rho += m_weights[i] * sin((iBand+1)*(_theta));
        }
    }
    if (rho < 0) {
        rho = 0;
    }
    return rho;
} // Lenth AdvancedCH::GetRho

Ray AdvancedCH::GetNormal(Radius theta) {
    Lenth rho = GetRho(theta);
    const Radius dTheta = 0.00001; // 因为极径应用了ReLU算子，故无法通过数学手段直接微分
    Lenth drho = GetRho(theta + dTheta) - rho;
    Lenth dRho_dTheta = drho / dTheta;
    Radius _theta = theta + m_rotation;
    Lenth dx_dTheta = dRho_dTheta * cos(_theta) - rho * sin(_theta);
    Lenth dy_dTheta = dRho_dTheta * sin(_theta) + rho * cos(_theta);
    Lenth l = sqrt(dx_dTheta * dx_dTheta + dy_dTheta * dy_dTheta);
    Vector2d norm;
    if (l == 0) {
        norm.x = cos(_theta); norm.y = sin(_theta);
    }
    norm.x = dy_dTheta / l, norm.y = -dx_dTheta / l;
    Ray norm1;
    norm1.start.x = m_position.x + rho * cos(theta);
    norm1.start.y = m_position.y + rho * sin(theta);
    norm1.direction.x = cos(m_rotation) * norm.x + sin(m_rotation) * norm.y;
    norm1.direction.y = -sin(m_rotation) * norm.x + cos(m_rotation) * norm.y;
    return norm1;
} // Vector2d AdvancedCH::GetNormal

bool AdvancedCH::CollideAdvancedCH(AdvancedCH* another) {
    Lenth dx = another->m_position.x - m_position.x;
    Lenth dy = another->m_position.y - m_position.y;
    Lenth norm = sqrt(dx*dx + dy*dy);
    Radius rad = acos(dx/norm);
    if (dy <= 0) rad = -rad;
    return norm <= GetRho(rad) + another->GetRho(rad+M_PI);
} // bool AdvancedCH::CollideAdvancedCH

void KineticCH::CalculateInertia() {
    const int max_segments = 1000;
    // 因为极径应用了ReLU算子，故无法通过数学手段直接积分，采用近似积分
    const Radius dTheta = 2 * M_PI / max_segments;
    Radius theta = 0;
    m_area = 0;
    m_inertiaMult = 0;
    for (int i = 0; i < max_segments; ++i) {
        Lenth rho = GetRho(theta);
        m_area += rho*rho*dTheta/2; // *dTheta/2 放到循环后执行能减少计算量，但可能导致溢出
        m_inertiaMult += rho * rho * rho * rho * dTheta / 4;
        theta += dTheta;
    }
    m_inertiaMult /= m_area;
} // void KineticCH::CalculateInertia

Vector2d KineticCH::GetSurfaceVelocity(Radius theta) {
    Ray norm = GetNormal(theta);
    return Vector2d{ m_velocity.x - norm.direction.y,
                     m_velocity.y + norm.direction.x };
} // Vector2d KineticCH::GetSurfaceVelocity

bool KineticCH::CollideKineticCH(KineticCH* another) {
    Lenth dx = another->m_position.x - m_position.x;
    Lenth dy = another->m_position.y - m_position.y;
    Lenth norm = sqrt(dx*dx + dy*dy);
    Radius rad = acos(dx/norm);
    if (dy <= 0) rad = -rad;
    Lenth rho0 = GetRho(rad);
    Lenth rho1 = another->GetRho(rad+M_PI);
    // 最速下降法求解最优化问题：
    // max(distProj) s.t. -π<offsetRad<π
    Radius offsetRad = 0; Lenth distProj = rho0 + rho1;
    const Lenth dRad = 0.001;
    const Lenth step = M_PI / 180;
    Lenth lastDistProj; Radius lastR1Rad; Radius lastOffsetRad;
    Radius r1Rad = rad + M_PI;
    for (;-M_PI < offsetRad && offsetRad < M_PI;) {
        lastDistProj = distProj; lastR1Rad = r1Rad; lastOffsetRad = offsetRad;
        rho0 = GetRho(rad+offsetRad+dRad);
        Vector2d p;
        p.x = m_position.x + rho0 * cos(rad+offsetRad+dRad);
        p.y = m_position.y + rho0 * sin(rad+offsetRad+dRad);
        Lenth _dx = p.x - another->m_position.x;
        Lenth _dy = p.y - another->m_position.y;
        Lenth _norm = sqrt(_dx*_dx+_dy*_dy);
        r1Rad = acos(_dx / _norm);
        if (_dy<0) r1Rad = -r1Rad;
        rho1 = another->GetRho(r1Rad);
        Lenth dDistProj = rho0 * cos(offsetRad+dRad) + rho1 * cos(r1Rad - rad - M_PI);
        dDistProj -= distProj;
        Lenth gradient = dDistProj / dRad;
        gradient = std::tanh(gradient);
        offsetRad += step * gradient;
        rho0 = GetRho(rad+offsetRad);
        p.x = m_position.x + rho0 * cos(rad+offsetRad);
        p.y = m_position.y + rho0 * sin(rad+offsetRad);
        _dx = p.x - another->m_position.x;
        _dy = p.y - another->m_position.y;
        _norm = sqrt(_dx*_dx+_dy*_dy);
        r1Rad = acos(_dx / _norm);
        if (_dy<0) r1Rad = -r1Rad;
        rho1 = another->GetRho(r1Rad);
        distProj = rho0 * cos(offsetRad) + rho1 * cos(r1Rad - rad - M_PI);
        if (distProj <= lastDistProj || (-dRad < gradient && gradient < dRad)) break;
    }
    if (norm > lastDistProj) {
        return false;
    }
//    Ray n0 = GetNormal(rad);
//    Ray n1 = another->GetNormal(rad+M_PI);
    Ray n0 = GetNormal(rad + lastOffsetRad);
    Ray n1 = another->GetNormal(lastR1Rad);
    Vector2d collisionPoint = Vector2d{ (n0.start.x + n1.start.x)/2,
                                        (n0.start.y + n1.start.y)/2 };
    Vector2d n1_ = Vector2d{ (n1.direction.x - n0.direction.x)/2,
                             (n1.direction.y - n0.direction.y)/2 };
    Lenth elastic = m_elastic * another->m_elastic;
    Vector2d r0 = Vector2d{ collisionPoint.x - m_position.x,
                            collisionPoint.y - m_position.y };
    Vector2d r1 = Vector2d{ collisionPoint.x - another->m_position.x,
                            collisionPoint.y - another->m_position.y };
    Lenth r0_n1 = r0.x * n1_.y - r0.y * n1_.x;
    Lenth r1_n1 = r1.x * n1_.y - r1.y * n1_.x;
    // j = -(1+e) * (v0·n1 - v1·n1 + ω0*r0×n1 - ω1*r0×n1) / (1/m0 + 1/m1 + (r0×n1)^2/I0 + (r1×n1)^2/I1)
    Lenth j;
    // 质量为负数代表无穷大
    if (another->m_mass <= 0) {
        j = - (1 + elastic) * ( m_velocity.x * n1_.x + m_velocity.y * n1_.y
                                - another->m_velocity.x * n1_.x - another->m_velocity.y * n1_.y
                                - m_omega * r0_n1 + another->m_omega * r1_n1 )
            / ( 1 / m_mass + r0_n1 * r0_n1 / (m_mass * m_inertiaMult) );
    } else {
        j = - (1 + elastic) * ( m_velocity.x * n1_.x + m_velocity.y * n1_.y
                            - another->m_velocity.x * n1_.x - another->m_velocity.y * n1_.y
//                            - m_omega * r0_n1 + another->m_omega * r0_n1 )
                            - m_omega * r0_n1 + another->m_omega * r1_n1 )
//                            + m_omega * r0_n1 - another->m_omega * r0_n1 )
//                            + m_omega * r0_n1 - another->m_omega * r1_n1 )
                        / ( 1 / m_mass + 1 / another->m_mass + r0_n1 * r0_n1 / (m_mass * m_inertiaMult)
                            + r1_n1 * r1_n1 / (another->m_mass * another->m_inertiaMult) );
    }
    if (j<=0) return false;
    Vector2d j0 = Vector2d{ j * n1_.x, j * n1_.y};
    // TODO: 计算摩擦冲量
//    Lenth jf = j * (m_friction + another->m_friction);
//    Vector2d jf0 = Vector2d{ jf * n1_.y, - jf * n1_.x};
//    j0.x += jf0.x; j0.y += jf0.y;
    // 应用冲量
    m_velocity.x += j0.x / m_mass;
    m_velocity.y += j0.y / m_mass;
    if (another->m_mass > 0) {
        another->m_velocity.x -= j0.x / another->m_mass;
        another->m_velocity.y -= j0.y / another->m_mass;
    }
    // 转动距 M = r × j;
    m_omega -= (r0.x * j0.y - r0.y * j0.x) / (m_mass * m_inertiaMult);
    if (another->m_mass > 0) {
        another->m_omega += (r1.x * j0.y - r1.y * j0.x) / (another->m_mass * another->m_inertiaMult);
    }
//    m_omega += (r0.x * j0.y - r0.y * j0.x) / (m_mass * m_inertiaMult);
//    another->m_omega -= (r1.x * j0.y - r1.y * j0.x) / (another->m_mass * another->m_inertiaMult);
    // 应用冲量 end
    return true;
} // void KineticCH::CollideKineticCH

void KineticCH::Move(float dTime) {
    m_position.x += m_velocity.x * dTime;
    m_position.y += m_velocity.y * dTime;
    m_rotation += m_omega * dTime;
} // void KineticCH::Move
} // namespace AdvancedCH