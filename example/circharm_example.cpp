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
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <SDL.h>
#include "circharm.hpp"

void renderCHInfo(CircHarm::CH& ch, int id) {
    ImGui::BeginGroup();
    ImGui::PushID(&ch);
    char label[20];
    sprintf(label, u8"ch %d:", id);
    ImGui::Text(label);
    ImGui::SliderFloat(u8" position x", &ch.position.x, -200.f, 200.f, "%.0f");
    ImGui::SliderFloat(u8" position y", &ch.position.y, -200.f, 200.f, "%.0f");
    ImGui::SliderFloat(u8" bias", &ch.bias, 0, 200.f, "%.0f");
    sprintf(label, u8" bands %d", id);
    ImGui::SliderInt(label, &ch.nBands, 0, 16);
    for (int i = 0; i < ch.nBands; ++i) {
        char _label[20];
        sprintf(_label, u8" weight %d", 2*i);
        ImGui::DragFloat(_label, &ch.weights[2*i], 1.0f, -200.f, 200.f, "%.0f");
        sprintf(_label, " weight %d", 2*i+1);
        ImGui::DragFloat(_label, &ch.weights[2*i+1], 1.0f, -200.f, 200.f, "%.0f");
    }
    ImGui::PopID();
    ImGui::EndGroup();
}

void RenderMultiCH(SDL_Renderer* renderer, CircHarm::MultiCH& multiCh) {
    std::vector<CircHarm::Vector2d> contour = CircHarm::GetContour(multiCh);
    SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, 0xff);
    for (auto& point : contour) {
        SDL_RenderDrawPointF(renderer, point.x+multiCh.position.x, point.y+multiCh.position.y);
    }
}

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

    // 定义MultiCH
    CircHarm::MultiCH multiCh = { 0 };
    multiCh.nKernels = 3;
    multiCh.position = CircHarm::Vector2d{ 3*width/4, height/2 };
    multiCh.rotate = 0;
    CircHarm::CH& ch0 = multiCh.kernels[0];
    ch0.position = CircHarm::Vector2d{ -100, -100 };
    ch0.nBands = 0;
    ch0.bias = 100;
    CircHarm::CH& ch1 = multiCh.kernels[1];
    ch1.position = CircHarm::Vector2d{ 100, -100 };
    ch1.nBands = 1;
    ch1.bias = 100;
    ch1.weights[0] = 50;
    ch1.weights[1] = 50;
    CircHarm::CH& ch2 = multiCh.kernels[2];
    ch2.position = CircHarm::Vector2d{ 1, 1 };
    ch2.nBands = 3;
    ch2.bias = 100;
    ch2.weights[0] = 0;
    ch2.weights[1] = 20;
    ch2.weights[2] = 20;
    ch2.weights[3] = 0;
    // 定义MultiCH end
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
            ImGui::Begin(u8"MultiCH");
            ImGui::BeginGroup();
            ImGui::SliderFloat(u8"position x", &multiCh.position.x, 0.f, width, "%.0f");
            ImGui::SliderFloat(u8"position y", &multiCh.position.y, 0.f, height, "%.0f");
            ImGui::SliderFloat(u8"rotation", &multiCh.rotate, -M_PI, M_PI, "%.02f");
            ImGui::EndGroup();
            renderCHInfo(ch0, 0);
            renderCHInfo(ch1, 1);
            renderCHInfo(ch2, 2);
            ImGui::End();
        } // IMGUI window
        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xff);
        SDL_RenderClear(renderer);
        RenderMultiCH(renderer, multiCh);
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