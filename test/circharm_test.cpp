// Copyright (c) [2024] [Initial Equationor]
// [circular harmonics] is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include "circharm.hpp"

TEST(CircHarm, CDVector2d2CH) {
    CircHarm::CH ch = { 0 };
    ch.position = CircHarm::Vector2d{2,2};
    ch.nBands = 0;
    ch.bias = 2;
    bool test_result[25] = { 0, 0, 1, 0, 0,
                             0, 1, 1, 1, 0,
                             1, 1, 1, 1, 1,
                             0, 1, 1, 1, 0,
                             0, 0, 1, 0, 0 };
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            auto p = CircHarm::Vector2d{ static_cast<CircHarm::Lenth>(x),
                                         static_cast<CircHarm::Lenth>(y) };
            ASSERT_EQ(test_result[5*y+x], CircHarm::CDVector2d2CH(p, ch)) << "(" << x << "," << y << ")";
        } // for x
    } // for y
}