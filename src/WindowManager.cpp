#include "WindowManager.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <string>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl3.h"
#include <stdio.h>
#include <SDL2/SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL2/SDL_opengles2.h>
#else
#include <SDL2/SDL_opengl.h>
#endif


WindowManager::~WindowManager()
{
    if (mWindowThread)
    {
        delete mWindowThread;
    }
}
void WindowManager::showWindow(std::map<uint32_t, rio_control_node::Motor_Config>& currMotorConfigMap, std::mutex& motorConfigMutex, std::map<uint32_t, rio_control_node::Motor_Info>& currMotorStatusMap, std::mutex& motorInfoMutex)
{
    if (!mWindowThread)
    {
        mWindowThread = new std::thread(&WindowManager::showWindowOpenGL3_internal, this);
        mMotorConfigMap = &currMotorConfigMap;
        mOutputMotorConfigMutex = &motorConfigMutex;
        mMotorStatusMap = &currMotorStatusMap;
        mMotorInfoMutex = &motorInfoMutex;
    }
}

std::thread* WindowManager::getThreadHandle()
{
    return mWindowThread;
}

bool WindowManager::isUpdateRequested()
{
    return mUpdateRequested;
}

void WindowManager::resetUpdateRequested()
{
    std::scoped_lock<std::mutex> lock(mUpdateMutex);
    mUpdateRequested = false;
}

void WindowManager::showWindowOpenGL3_internal()
{
    // Setup SDL
    // (Some versions of SDL before <2.0.10 appears to have performance/stalling issues on a minority of Windows systems,
    // depending on whether SDL_INIT_GAMECONTROLLER is enabled or disabled.. updating to latest version of SDL is recommended!)
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return;
    }

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(__APPLE__)
    // GL 3.2 Core + GLSL 150
    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

    // Create window with graphics context
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("PID Tuner | CyberKnights Team 195", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    while (!done)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        captureTunableMotors();

        for (const std::pair<uint32_t, rio_control_node::Motor_Config>& m : mTrackedMotors)
        {
            drawImguiWindow(m.first, std::distance(mTrackedMotors.begin(),mTrackedMotors.find(m.first)));
        }

        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    ros::shutdown();

    return;
}

void WindowManager::drawImguiWindow(int motorId, int listPos)
{
    int windowWidth = 200;
    int windowHeight = 400;
    ImGui::SetNextWindowPos(ImVec2((0 + (listPos-1) * windowWidth),0 ));
    ImGui::SetNextWindowSize(ImVec2(windowWidth, windowHeight), ImGuiCond_Always);

    std::string widgetTitle = "PID Tuner ";
    widgetTitle += std::to_string(motorId);
    ImGui::Begin(widgetTitle.c_str());   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
    
    if (mMotorConfigMap->size() > 0)
    {
        static const char* current_motor = NULL;
        if(ImGui::BeginCombo("Motor", current_motor))
        {
            for (auto it = mMotorConfigMap->begin(); it != mMotorConfigMap->end(); it++)
            {
                bool is_selected = (std::string(current_motor) == std::to_string(it->first)); // You can store your selection however you want, outside or inside your objects
                if (ImGui::Selectable(std::to_string(it->first).c_str(), is_selected))
                {
                    current_motor = std::to_string(it->first).c_str();
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
                }
            }
            ImGui::EndCombo();
        }
    }

    double kP;
    ImGui::InputDouble("kP", &kP, 0.001, 0.1);
    if (ImGui::Button("Send Update"))
    {
        std::scoped_lock<std::mutex> lock(mUpdateMutex);
        mUpdateRequested = true;
    }
    ImGui::End();
}

void WindowManager::captureTunableMotors()
{
    for (const std::pair<uint32_t, rio_control_node::Motor_Config>& m : *mMotorConfigMap)
    {
        if (m.second.controller_mode == rio_control_node::Motor_Config::MASTER
            || m.second.controller_mode == rio_control_node::Motor_Config::FAST_MASTER)
        {
            mTrackedMotors[m.first] = m.second;
        }
    }
}