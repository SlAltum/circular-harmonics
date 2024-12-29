// Copyright (c) [2024] [Initial Equationor]
// [circular harmonics] is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <vector>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <SDL.h>
#include "advanced_ch.hpp"
#define WIDTH 960
#define HEIGHT 540

class SDLCircHarm : public AdvancedCH::KineticCH {
public:
    SDL_Color m_color;
    SDLCircHarm() {}
    SDLCircHarm* Clone() const override {
        return new SDLCircHarm(*this);
    }
    void Reset() override {
        KineticCH::Reset();
    }
    void Render(SDL_Renderer* renderer) {
        SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, 0xff);
        const int nVerts = 360;
        const ::AdvancedCH::Radius dRad = 2*M_PI/nVerts;
        ::AdvancedCH::Radius rad = 0;
        SDL_FPoint points[nVerts+1];
        ::AdvancedCH::Lenth rho = GetRho(rad);
        points[0] = SDL_FPoint{ m_position.x + rho, m_position.y };
        points[nVerts] = SDL_FPoint{ m_position.x + rho, m_position.y };
        rad += dRad;
        for (int i = 0; i < nVerts; ++i) {
            ::AdvancedCH::Lenth rho = GetRho(rad);
            points[i] = SDL_FPoint{ m_position.x + rho * cos(rad),
                                    m_position.y + rho * sin(rad) };
            rad += dRad;
        }
        SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
        SDL_RenderDrawLinesF(renderer, points, nVerts+1);
        SDL_RenderDrawLine(renderer, m_position.x, m_position.y,
                           m_position.x + 10 * cos(m_rotation),
                           m_position.y - 10 * sin(m_rotation));
        SDL_RenderDrawLine(renderer, m_position.x, m_position.y,
                           m_position.x + 10 * cos(m_rotation+M_PI/2),
                           m_position.y - 10 * sin(m_rotation+M_PI/2));
    } // void Render
}; // class SDLCircHarm

void renderCHInfo(SDLCircHarm& ch, int id) {
    ImGui::BeginGroup();
    ImGui::PushID(&ch);
    char label[20];
    sprintf(label, u8"ch %d:", id);
    ImGui::Text(label);
    ImGui::SliderFloat(u8" position x", &ch.m_position.x, 0.f, WIDTH, "%.0f");
    ImGui::SliderFloat(u8" position y", &ch.m_position.y, 0.f, HEIGHT, "%.0f");
    ImGui::DragFloat(u8" rotation", &ch.m_rotation, 0.01f, -M_PI, M_PI, "%.02f");
    ImGui::SliderFloat(u8" mass", &ch.m_mass, 0.f, 100.f, "%.0f");
    ch.CalculateInertia();
    ImGui::SliderFloat(u8" velocity x", &ch.m_velocity.x, -100.f, 100.f, "%.0f");
    ImGui::SliderFloat(u8" velocity y", &ch.m_velocity.y, -100.f, 100.f, "%.0f");
    ImGui::DragFloat(u8" omega", &ch.m_omega, 0.01f, -2*M_PI, 2*M_PI, "%.02f");
    ImGui::DragFloat(u8" bias", &ch.m_bias, 1.f, 0.f, 1000.f, "%.0f");
    ImGui::DragInt(u8" n wights", &ch.m_nWeights, 1.0f, 0, CH_MAX_WEIGHTS);
    for (int i = 0; i < ch.m_nWeights; ++i) {
        char _label[20];
        sprintf(_label, u8" iWeight %d", i);
        ImGui::DragInt(_label, &ch.m_iWeights[i], 1.0f, 0, 1000);
        sprintf(_label, " weight %d", i);
        ImGui::DragFloat(_label, &ch.m_weights[i], 1.0f, -500.f, 500.f, "%.0f");
    }
    ImGui::PopID();
    ImGui::EndGroup();
}

void FrameProcessCH(SDL_Renderer* renderer, SDLCircHarm &ch1, SDLCircHarm &ch2) {
    ch1.Render(renderer);
    ch2.Render(renderer);
    // 碰撞
    AdvancedCH::Lenth dx = ch2.m_position.x - ch1.m_position.x;
    AdvancedCH::Lenth dy = ch2.m_position.y - ch1.m_position.y;
    AdvancedCH::Lenth norm = sqrt(dx*dx + dy*dy);
    AdvancedCH::Radius rad = acos(dx/norm);
    if (dy <= 0) rad = -rad;
    AdvancedCH::Lenth rho0 = ch1.GetRho(rad);
    AdvancedCH::Lenth rho1 = ch2.GetRho(rad+M_PI);
    // 最速下降法求解最优化问题：
    // max(distProj) s.t. -π<offsetRad<π
    AdvancedCH::Radius offsetRad = 0; AdvancedCH::Lenth distProj = rho0 + rho1;
    const AdvancedCH::Lenth dRad = 0.001;
    const AdvancedCH::Lenth step = M_PI / 180;
    AdvancedCH::Lenth lastDistProj; AdvancedCH::Radius lastR1Rad; AdvancedCH::Radius lastOffsetRad;
    AdvancedCH::Radius r1Rad = rad + M_PI;
    for (;-M_PI < offsetRad && offsetRad < M_PI;) {
        lastDistProj = distProj; lastR1Rad = r1Rad; lastOffsetRad = offsetRad;
        rho0 = ch1.GetRho(rad+offsetRad+dRad);
        AdvancedCH::Vector2d p;
        p.x = ch1.m_position.x + rho0 * cos(rad+offsetRad+dRad);
        p.y = ch1.m_position.y + rho0 * sin(rad+offsetRad+dRad);
        AdvancedCH::Lenth _dx = p.x - ch2.m_position.x;
        AdvancedCH::Lenth _dy = p.y - ch2.m_position.y;
        AdvancedCH::Lenth _norm = sqrt(_dx*_dx+_dy*_dy);
        r1Rad = acos(_dx / _norm);
        if (_dy<0) r1Rad = -r1Rad;
        rho1 = ch2.GetRho(r1Rad);
        AdvancedCH::Lenth dDistProj = rho0 * cos(offsetRad+dRad) + rho1 * cos(r1Rad - rad - M_PI);
        dDistProj -= distProj;
        AdvancedCH::Lenth gradient = dDistProj / dRad;
        gradient = std::tanh(gradient);
        offsetRad += step * gradient;
        rho0 = ch1.GetRho(rad+offsetRad);
        p.x = ch1.m_position.x + rho0 * cos(rad+offsetRad);
        p.y = ch1.m_position.y + rho0 * sin(rad+offsetRad);
        _dx = p.x - ch2.m_position.x;
        _dy = p.y - ch2.m_position.y;
        _norm = sqrt(_dx*_dx+_dy*_dy);
        r1Rad = acos(_dx / _norm);
        if (_dy<0) r1Rad = -r1Rad;
        rho1 = ch2.GetRho(r1Rad);
        distProj = rho0 * cos(offsetRad) + rho1 * cos(r1Rad - rad - M_PI);
        if (distProj <= lastDistProj || gradient == 0) break;
    }
    if (norm > lastDistProj) {
        return;
    }
//    if (norm > rho0 + rho1) {
//        return;
//    }
//    AdvancedCH::Ray n0 = ch1.GetNormal(rad);
//    AdvancedCH::Ray n1 = ch2.GetNormal(rad+M_PI);
    AdvancedCH::Ray n0 = ch1.GetNormal(rad + lastOffsetRad);
    AdvancedCH::Ray n1 = ch2.GetNormal(lastR1Rad);
    // 绘制法向量
    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0, 0xff);
    const int arrowLen = 15;
    SDL_RenderDrawLine(renderer, n0.start.x, n0.start.y, n0.start.x + arrowLen * n0.direction.x, n0.start.y + arrowLen * n0.direction.y);
    SDL_RenderDrawLine(renderer, n0.start.x, n0.start.y, n0.start.x - arrowLen * n0.direction.y, n0.start.y + arrowLen * n0.direction.x);
    SDL_RenderDrawLine(renderer, n0.start.x, n0.start.y, n0.start.x + arrowLen * n0.direction.y, n0.start.y - arrowLen * n0.direction.x);
    SDL_RenderDrawLine(renderer, n1.start.x, n1.start.y, n1.start.x + arrowLen * n1.direction.x, n1.start.y + arrowLen * n1.direction.y);
    SDL_RenderDrawLine(renderer, n1.start.x, n1.start.y, n1.start.x - arrowLen * n1.direction.y, n1.start.y + arrowLen * n1.direction.x);
    SDL_RenderDrawLine(renderer, n1.start.x, n1.start.y, n1.start.x + arrowLen * n1.direction.y, n1.start.y - arrowLen * n1.direction.x);
    // 绘制法向量 end
    AdvancedCH::Vector2d collisionPoint = AdvancedCH::Vector2d{ (n0.start.x + n1.start.x)/2,
                                        (n0.start.y + n1.start.y)/2 };
    AdvancedCH::Vector2d n1_ = AdvancedCH::Vector2d{ (n1.direction.x - n0.direction.x)/2,
                             (n1.direction.y - n0.direction.y)/2 };
    AdvancedCH::Lenth elastic = ch1.m_elastic * ch2.m_elastic;
    AdvancedCH::Vector2d r0 = AdvancedCH::Vector2d{ collisionPoint.x - ch1.m_position.x,
                                                    collisionPoint.y - ch1.m_position.y };
    AdvancedCH::Vector2d r1 = AdvancedCH::Vector2d{ collisionPoint.x - ch2.m_position.x,
                                                    collisionPoint.y - ch2.m_position.y };
    AdvancedCH::Lenth r0_n1 = r0.x * n1_.y - r0.y * n1_.x;
    AdvancedCH::Lenth r1_n1 = r1.x * n1_.y - r1.y * n1_.x;
    // j = -(1+e) * (v0·n1 - v1·n1 + ω0*r0×n1 - ω1*r0×n1) / (1/m0 + 1/m1 + (r0×n1)^2/I0 + (r1×n1)^2/I1)
    AdvancedCH::Lenth j = - (1 + elastic) * ( ch1.m_velocity.x * n1_.x + ch1.m_velocity.y * n1_.y
                                            - ch2.m_velocity.x * n1_.x - ch2.m_velocity.y * n1_.y
                                            - ch1.m_omega * r0_n1 + ch2.m_omega * r1_n1 )
//                                            + ch1.m_omega * r0_n1 - ch2.m_omega * r0_n1 )
//                                            + ch1.m_omega * r0_n1 - ch2.m_omega * r1_n1 )
                            / ( 1 / ch1.m_mass + 1 / ch2.m_mass
                                + r0_n1 * r0_n1 / (ch1.m_mass * ch1.m_inertiaMult)
                                + r1_n1 * r1_n1 / (ch2.m_mass * ch2.m_inertiaMult) );
    j = j > 0 ? j : 0;
    AdvancedCH::Vector2d j0 = AdvancedCH::Vector2d{ j * n1_.x, j * n1_.y};
    // 摩擦冲量
//    AdvancedCH::Lenth jf = j * (ch1.m_friction + ch2.m_friction);
//    AdvancedCH::Vector2d jf0 = AdvancedCH::Vector2d{ jf * n1_.y, - jf * n1_.x};
//    j0.x += jf0.x; j0.y += jf0.y;
    // 绘制动量
    const int momentumArrowLen = 5;
    SDL_RenderDrawLine(renderer, ch1.m_position.x, ch1.m_position.y,
                       ch1.m_position.x + momentumArrowLen * ch1.m_mass * ch1.m_velocity.x,
                       ch1.m_position.y + momentumArrowLen * ch1.m_mass * ch1.m_velocity.y);
    SDL_RenderDrawLine(renderer, ch2.m_position.x, ch2.m_position.y,
                       ch2.m_position.x + momentumArrowLen * ch2.m_mass * ch2.m_velocity.x,
                       ch2.m_position.y + momentumArrowLen * ch2.m_mass * ch2.m_velocity.y);
    SDL_SetRenderDrawColor(renderer, 0, 0xff, 0, 0xff);
    // 绘制冲量
    SDL_RenderDrawLine(renderer, ch1.m_position.x, ch1.m_position.y,
                       ch1.m_position.x + momentumArrowLen * j0.x, ch1.m_position.y + momentumArrowLen * j0.y);
    SDL_RenderDrawLine(renderer, ch2.m_position.x, ch2.m_position.y,
                       ch2.m_position.x - momentumArrowLen * j0.x, ch2.m_position.y - momentumArrowLen * j0.y);
    // 绘制冲量 end
    // TODO: 计算摩擦冲量
//    // 应用冲量
//    ch1.m_velocity.x += j0.x / ch1.m_mass;
//    ch1.m_velocity.y += j0.y / ch1.m_mass;
//    ch2.m_velocity.x -= j0.x / ch2.m_mass;
//    ch2.m_velocity.y -= j0.y / ch2.m_mass;
//    // 转动距 M = r × j;
//    ch1.m_omega += (r0.x * j0.y - r0.y * j0.x) / (ch1.m_mass * ch1.m_inertiaMult);
//    ch2.m_omega -= (r1.x * j0.y - r1.y * j0.x) / (ch2.m_mass * ch2.m_inertiaMult);
} // void FrameProcessCH

int main(int argc, char *argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }
    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif
    const int width = 960, height = 540;
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("SDLmap",
                                          SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, window_flags);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return -1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr)
    {
        SDL_Log("Error creating SDL_Renderer!");
        return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    ImFont* font = io.Fonts->AddFontFromFileTTF("assets/AlibabaPuHuiTi-3-55-Regular.ttf", 22.0f, nullptr, io.Fonts->GetGlyphRangesChineseFull());
    IM_ASSERT(font != nullptr);

    // 定义CircHarm
    SDLCircHarm ch1;
    ch1.m_position.x = 400;
    ch1.m_position.y = 270;
    ch1.m_bias = 100;
    ch1.m_nWeights = 0;
    ch1.m_mass = 10;
    ch1.CalculateInertia();
    ch1.m_color = SDL_Color{ 0xff, 0, 0, 0xff };
    SDLCircHarm ch2;
    ch2.m_position.x = 555;
    ch2.m_position.y = 270;
    ch2.m_bias = 100;
    ch2.m_nWeights = 1;
    ch2.m_iWeights[0] = 4;
    ch2.m_weights[0] = 50;
    ch2.m_mass = 10;
    ch2.CalculateInertia();
    ch2.m_color = SDL_Color{ 0xff, 0, 0, 0xff };
    // 定义CircHarm end
    for(bool shouldQuit = false;!shouldQuit;) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            switch (event.type) {
                case SDL_QUIT:
                    shouldQuit = true;
            }
        }
        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        {
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);
            ImGui::Begin(u8"ch collision");
            renderCHInfo(ch1, 1);
            renderCHInfo(ch2, 2);
            ImGui::End();
        } // IMGUI window
        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xff);
        SDL_RenderClear(renderer);
        FrameProcessCH(renderer, ch1, ch2);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    } // main loop
    // Cleanup
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}