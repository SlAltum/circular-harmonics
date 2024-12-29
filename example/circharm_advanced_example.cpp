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


class CHPool
{
private:
    std::vector<SDLCircHarm*> m_prototypes;
    std::list<SDLCircHarm*> m_ch;
    CHPool() {
        auto prototype0 = new SDLCircHarm();
        prototype0->m_bias = 32;
        prototype0->m_nWeights = 0;
        prototype0->m_mass = 10;
        prototype0->CalculateInertia();
        m_prototypes.push_back(prototype0);
        auto prototype1 = new SDLCircHarm();
        prototype1->m_bias = 32;
        prototype1->m_nWeights = 1;
        prototype1->m_iWeights[0] = 2;
        prototype1->m_weights[0] = 16;
        prototype1->m_mass = 10;
        prototype1->CalculateInertia();
        m_prototypes.push_back(prototype1);
        auto prototype2 = new SDLCircHarm();
        prototype2->m_bias = 32;
        prototype2->m_nWeights = 1;
        prototype2->m_iWeights[0] = 6;
        prototype2->m_weights[0] = 16;
        prototype2->m_mass = 10;
        prototype2->CalculateInertia();
        m_prototypes.push_back(prototype2);
    };
    ~CHPool() = default;
public:
    CHPool(const CHPool&) = delete;
    CHPool& operator=(const CHPool&) = delete;
    static CHPool& getInstance() {
        static CHPool instance;
        return instance;
    }
    SDLCircHarm* get()
    {
        SDLCircHarm* ch;
        srand(SDL_GetTicks());
        if (m_ch.empty())
        {
            ch = m_prototypes[rand() % m_prototypes.size()]->Clone();
        }
        else
        {
            ch = m_ch.front();
            m_ch.pop_front();
        }
        ch = m_prototypes[rand() % m_prototypes.size()]->Clone();
        ch->m_color.r = 64 + rand() % 192;
        ch->m_color.g = 64 + rand() % 192;
        ch->m_color.b = 64 + rand() % 192;
        ch->m_color.a = 0xff;
        ch->m_omega = 0;
//        ch->m_omega = 2 * M_PI * (rand() % 256) / 256;
        return ch;
    }
    void returnCH(SDLCircHarm* ch)
    {
//        ch->Reset();
        m_ch.push_back(ch);
    }
}; // class CHPool

const AdvancedCH::Lenth borderL = 300;
const AdvancedCH::Lenth borderR = 960;
const AdvancedCH::Lenth borderT = 0;
const AdvancedCH::Lenth borderB = 540;

void SpawnCH(std::list<SDLCircHarm *> &ch_list, SDL_Event& event) {
    const static Uint32 MinInternal = 500; // 1s
    static Uint32 stopwatch = SDL_GetTicks();
    Uint32 now = SDL_GetTicks();
    if (now - stopwatch < MinInternal) return;
    stopwatch = now;
    if (event.button.x < borderL || event.button.x > borderR ||
        event.button.y < borderT || event.button.y > borderB) {
        return;
    }
    SDLCircHarm* ch = CHPool::getInstance().get();
    ch->m_position.x = event.button.x;
    ch->m_position.y = borderT + 100;
    ch->m_velocity.x = 0;
    ch->m_velocity.y = 0;
    ch->m_elastic = 0.8;
//    ch->m_position.x = borderL + 100;
//    ch->m_position.y = borderT + 100;
//    ch->m_velocity.x = event.button.x - ch->m_position.x;
//    ch->m_velocity.y = event.button.y - ch->m_position.y;
//    AdvancedCH::Lenth vNorm = sqrt(ch->m_velocity.x * ch->m_velocity.x
//                                 + ch->m_velocity.y * ch->m_velocity.y);
//    srand(SDL_GetTicks());
//    AdvancedCH::Lenth velocity = 50 + 100 * (rand() % 256) / 256;
//    ch->m_velocity.x *= velocity / vNorm;
//    ch->m_velocity.y *= velocity / vNorm;
    ch_list.push_back(ch);
}

std::vector<SDLCircHarm *> obstacles;
void InitObs() {
    auto prototype = new SDLCircHarm();
    prototype->m_bias = 32;
    prototype->m_nWeights = 4;
    prototype->m_iWeights[0] = 6; prototype->m_weights[0] = -5.12;
    prototype->m_iWeights[1] = 14; prototype->m_weights[1] = 2.56;
    prototype->m_iWeights[2] = 30; prototype->m_weights[2] = -1.28;
    prototype->m_iWeights[3] = 62; prototype->m_weights[2] = 0.64;
    prototype->m_mass = -1;
    prototype->m_color = { 0xff, 0xff, 0xff, 0xff };
    prototype->m_elastic = 0.2;
    prototype->CalculateInertia();
    for (float x = borderL+32; x < borderR; x += 64) {
        auto obs = prototype->Clone();
        obs->m_position.x = x;
        obs->m_position.y = borderB-32;
        obstacles.push_back(obs);
    }
}

AdvancedCH::Lenth gravity = 9.8;
void FrameProcessCH(SDL_Renderer* renderer, std::list<SDLCircHarm *> &ch_list) {
    // 绘制边界
    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
    SDL_Rect border; border.x = borderL; border.y = borderT;
    border.w = borderR - borderL; border.h = borderB - borderT;
    SDL_RenderDrawRect(renderer, &border);
    for (auto& obs : obstacles) {
        obs->Render(renderer);
    }
    // 绘制边界 end
    static Uint32 stopwatch = SDL_GetTicks();
    Uint32 now = SDL_GetTicks();
    Uint32 elapse = now - stopwatch;
    stopwatch = now;
    float elapseF = static_cast<float>(elapse) / 1000;
    // 运动和回收
    for (auto it = ch_list.begin(); it != ch_list.end();) {
        (*it)->m_velocity.y += gravity * elapseF;
        (*it)->Move(elapseF);
        if ((*it)->m_position.x - (*it)->GetRho(M_PI)    > borderR ||
            (*it)->m_position.x + (*it)->GetRho(0)       < borderL ||
            (*it)->m_position.y - (*it)->GetRho(-M_PI/2) < borderT ||
            (*it)->m_position.y + (*it)->GetRho(M_PI/2)  > borderB){
            CHPool::getInstance().returnCH(*it);
            it = ch_list.erase(it);
        } else {
            it++;
        }
    }
    // 碰撞
    for (auto it = ch_list.begin(); it != ch_list.end(); it++) {
        auto jt = it;
        for (++jt; jt != ch_list.end(); jt++) {
            (*it)->CollideKineticCH(*jt);
        }
    }
    for (auto& ch : ch_list) {
        for (auto& obs : obstacles) {
            ch->CollideKineticCH(obs);
        }
    }
    // 渲染
    for (auto& ch : ch_list) {
        ch->Render(renderer);
    }
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
    InitObs();
    std::list<SDLCircHarm*> ch_list;
    // 定义CircHarm end
    for(bool shouldQuit = false;!shouldQuit;) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            switch (event.type) {
                case SDL_QUIT:
                    shouldQuit = true;
                case SDL_MOUSEBUTTONDOWN:
                    SpawnCH(ch_list, event);
            }
        }
        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        {
//            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
//            ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_Always);
//            ImGui::Begin(u8"MultiCH");
//            ImGui::Text(u8"epoch:%d",epoch);
//            ImGui::BeginGroup();
//            ImGui::SliderFloat(u8"position x", &multiCh.position.x, 0.f, width, "%.0f");
//            ImGui::SliderFloat(u8"position y", &multiCh.position.y, 0.f, height, "%.0f");
//            ImGui::SliderFloat(u8"rotation", &multiCh.rotate, -M_PI, M_PI, "%.02f");
//            ImGui::EndGroup();
//            renderCHInfo(ch0, 0);
//            ImGui::End();
        } // IMGUI window
        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xff);
        SDL_RenderClear(renderer);
        FrameProcessCH(renderer, ch_list);
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