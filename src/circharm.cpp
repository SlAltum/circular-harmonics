// Copyright (c) [2024] [Initial Equationor]
// [circular harmonics] is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
#include "circharm.hpp"
#include <stdexcept>

namespace CircHarm {
Lenth calculateDist(Vector2d& a, Vector2d& b) {
    Lenth dx = a.x - b.x;
    Lenth dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
} // Lenth calculateDist

Vector2d rotate(Vector2d& a, Radius rad) {
    return Vector2d{ static_cast<Lenth>(cos(rad)*a.x - sin(rad)*a.y),
                     static_cast<Lenth>(sin(rad)*a.x + cos(rad)*a.y) };
}

Lenth CalculateR(CH& ch, Radius rad) {
    Lenth r = ch.bias;
    for (int i = 0; i < ch.nBands; ++i) {
        r += ch.weights[2*i] * cos((i+1) * rad);
        r += ch.weights[2*i+1] * sin((i+1) * rad);
    }
    r = r>0 ? r : 0;
    return r;
} // Lenth CalculateR

Lenth calculateDiff(CH& ch, Radius rad) {
    const Lenth da = 0.00001;
    Lenth dr = CalculateR(ch, rad + da) - CalculateR(ch, rad);
    Lenth dr_da = dr/da;
//    Lenth dr_da = 0;
//    for (int i = 0; i < ch.nBands; ++i) {
//        dr_da -= ch.weights[2*i] * (i+1) * sin((i+1) * rad);
//        dr_da += ch.weights[2*i+1] * (i+1) * cos((i+1) * rad);
//    }
    return dr_da;
} // Lenth calculateDiff

Vector2d CalculateNorm(CH& ch, Radius rad) {
    Lenth r = CalculateR(ch, rad);
    Lenth dr_da = calculateDiff(ch, rad);
    Lenth dx_da = dr_da * cos(rad) - r * sin(rad);
    Lenth dy_da = dr_da * sin(rad) + r * cos(rad);
    Lenth l = sqrt(dx_da*dx_da+dy_da*dy_da);
    if (l == 0) {
        return Vector2d{ static_cast<Lenth>(cos(rad)),
                         static_cast<Lenth>(sin(rad)) };
    }
    return Vector2d{ dy_da/l, -dx_da/l };
} // Vector2d CalculateNorm

Lenth CalculateArea(CH& ch) {
    const int nSegments = 1000;
    Lenth area = 0;
    Lenth rad = 0;
    Lenth dRad = 2 * M_PI / nSegments;
    for (int i = 0; i < nSegments; ++i) {
        Lenth r = CalculateR(ch, rad);
        area += r * r * dRad / 2;
        rad += dRad;
    }
    return area;
}

bool CDVector2d2CH(Vector2d p, CH ch) {
    Lenth dx = p.x - ch.position.x;
    Lenth dy = p.y - ch.position.y;
    if (dx == 0 && dy == 0) return true;
    Lenth dist = calculateDist(p, ch.position);
    Radius rad = acos(dx/dist);
    if (dy<0) {
        rad = -rad;
    }
    return dist <= CalculateR(ch, rad);
}

std::vector<Vector2d> GetContour(MultiCH shape) {
    if (shape.nKernels < 1) {
        throw std::runtime_error("[CircHarm]getContour: MultiCH must has at least one kernel.");
    }
    Radius rad = -M_PI;
    const int nPoints = 360;
    Radius dRad = 2*M_PI/nPoints;
    std::vector<Vector2d> contour;
    for (int i = 0; i < nPoints; ++i) {
        if (1 == shape.nKernels) {
            CH& _kernel = shape.kernels[0];
            Lenth _r = CalculateR(_kernel, rad);
            auto p = Vector2d{ static_cast<Lenth>(_kernel.position.x + _r * cos(rad)),
                               static_cast<Lenth>(_kernel.position.y + _r * sin(rad)) };
            contour.push_back(rotate(p, shape.rotate));
        }
        for (int j = 0; j < shape.nKernels; ++j) {
            CH& _kernel = shape.kernels[j];
            Lenth _r = CalculateR(_kernel, rad);
            auto p = Vector2d{ static_cast<Lenth>(_kernel.position.x + _r * cos(rad)),
                               static_cast<Lenth>(_kernel.position.y + _r * sin(rad)) };
            bool isOnContour = true;
            for (int k = 0; k < shape.nKernels-1; ++k) {
                CH& _kernel2 = shape.kernels[(j+k+1)%shape.nKernels];
                if (CDVector2d2CH(p, _kernel2)) {
                    isOnContour = false;
                    break;
                }
            }
            if (isOnContour) {
                contour.push_back(rotate(p, shape.rotate));
            }
        } // for j
        rad += dRad;
    } // for i
    return contour;
} // std::vector<Vector2d> getContour
} // namespace CircHarm