#include "WindowManager.hpp"
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <string>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl3.h"

#include "ros/ros.h"

#include <stdio.h>
#include <SDL2/SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL2/SDL_opengles2.h>
#else
#include <SDL2/SDL_opengl.h>
#endif

extern ros::NodeHandle* node;

WindowManager::~WindowManager()
{
    if (mWindowThread)
    {
        delete mWindowThread;
    }
}

void WindowManager::showWindow(std::map<uint32_t, rio_control_node::Motor_Config>& currMotorConfigMap, std::mutex& motorConfigMutex, std::map<uint32_t, rio_control_node::Motor_Info>& currMotorStatusMap, std::mutex& motorInfoMutex, std::map<uint32_t, rio_control_node::Motor>& currMotorControlMap, std::mutex& motorControlMutex)
{
    if (!mWindowThread)
    {
        mWindowThread = new std::thread(&WindowManager::showWindowOpenGL3_internal, this);
        mMotorConfigMap = &currMotorConfigMap;
        mOutputMotorConfigMutex = &motorConfigMutex;
        mMotorStatusMap = &currMotorStatusMap;
        mMotorInfoMutex = &motorInfoMutex;
        mMotorControlMap = &currMotorControlMap;
        mMotorControlMutex = &motorControlMutex;
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

static constexpr int WINDOW_WIDTH = 1280;
static constexpr int WINDOW_HEIGHT = 720;

void WindowManager::showWindowOpenGL3_internal()
{
    std::cout.setstate(std::ios::failbit);  //Block console output for unwanted init info
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
    SDL_Window* window = SDL_CreateWindow("PID Tuner | CyberKnights Team 195", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, window_flags);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    //Set Window Icon
    SDL_Surface *surface;     // Declare an SDL_Surface to be filled in with pixel data from an image file
    Uint16 pixels[16*16] = {  // ...or with raw pixel data:
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0aab, 0x0789, 0x0bcc, 0x0eee, 0x09aa, 0x099a, 0x0ddd,
        0x0fff, 0x0eee, 0x0899, 0x0fff, 0x0fff, 0x1fff, 0x0dde, 0x0dee,
        0x0fff, 0xabbc, 0xf779, 0x8cdd, 0x3fff, 0x9bbc, 0xaaab, 0x6fff,
        0x0fff, 0x3fff, 0xbaab, 0x0fff, 0x0fff, 0x6689, 0x6fff, 0x0dee,
        0xe678, 0xf134, 0x8abb, 0xf235, 0xf678, 0xf013, 0xf568, 0xf001,
        0xd889, 0x7abc, 0xf001, 0x0fff, 0x0fff, 0x0bcc, 0x9124, 0x5fff,
        0xf124, 0xf356, 0x3eee, 0x0fff, 0x7bbc, 0xf124, 0x0789, 0x2fff,
        0xf002, 0xd789, 0xf024, 0x0fff, 0x0fff, 0x0002, 0x0134, 0xd79a,
        0x1fff, 0xf023, 0xf000, 0xf124, 0xc99a, 0xf024, 0x0567, 0x0fff,
        0xf002, 0xe678, 0xf013, 0x0fff, 0x0ddd, 0x0fff, 0x0fff, 0xb689,
        0x8abb, 0x0fff, 0x0fff, 0xf001, 0xf235, 0xf013, 0x0fff, 0xd789,
        0xf002, 0x9899, 0xf001, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0xe789,
        0xf023, 0xf000, 0xf001, 0xe456, 0x8bcc, 0xf013, 0xf002, 0xf012,
        0x1767, 0x5aaa, 0xf013, 0xf001, 0xf000, 0x0fff, 0x7fff, 0xf124,
        0x0fff, 0x089a, 0x0578, 0x0fff, 0x089a, 0x0013, 0x0245, 0x0eff,
        0x0223, 0x0dde, 0x0135, 0x0789, 0x0ddd, 0xbbbc, 0xf346, 0x0467,
        0x0fff, 0x4eee, 0x3ddd, 0x0edd, 0x0dee, 0x0fff, 0x0fff, 0x0dee,
        0x0def, 0x08ab, 0x0fff, 0x7fff, 0xfabc, 0xf356, 0x0457, 0x0467,
        0x0fff, 0x0bcd, 0x4bde, 0x9bcc, 0x8dee, 0x8eff, 0x8fff, 0x9fff,
        0xadee, 0xeccd, 0xf689, 0xc357, 0x2356, 0x0356, 0x0467, 0x0467,
        0x0fff, 0x0ccd, 0x0bdd, 0x0cdd, 0x0aaa, 0x2234, 0x4135, 0x4346,
        0x5356, 0x2246, 0x0346, 0x0356, 0x0467, 0x0356, 0x0467, 0x0467,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff,
        0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff
    };
    surface = SDL_CreateRGBSurfaceFrom(pixels,16,16,16,16*2,0x0f00,0x00f0,0x000f,0xf000);
    // The icon is attached to the window pointer
    SDL_SetWindowIcon(window, surface);
    // ...and the surface containing the icon pixel data is no longer required.
    SDL_FreeSurface(surface);

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
    ros::Rate rate(100);
    while (!done && ros::ok())
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

        std::cout.clear();

        captureTunableMotors();

        for (const std::pair<uint32_t, rio_control_node::Motor_Config>& m : mTrackedMotorConfig)
        {
            drawImguiWindow(m.first, std::distance(mTrackedMotorConfig.begin(),mTrackedMotorConfig.find(m.first)));
        }

        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
        rate.sleep();
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
    int windowWidth = 250;
    int windowHeight = 400;
    const int maxNumWidgetsWide = (WINDOW_WIDTH / windowWidth);
    if (listPos > maxNumWidgetsWide - 1)
    {
        ImGui::SetNextWindowPos(ImVec2((listPos - maxNumWidgetsWide) * windowWidth, windowHeight));
    }
    else
    {
        ImGui::SetNextWindowPos(ImVec2(listPos * windowWidth, 0));
    }
    ImGui::SetNextWindowSize(ImVec2(windowWidth, windowHeight), ImGuiCond_Always);

    std::string widgetTitle = "PID Tuner ";
    widgetTitle += std::to_string(motorId);
    ImGui::Begin(widgetTitle.c_str());   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)

    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.45f);
    ImGui::InputDouble("kP", &mTrackedMotorConfig[motorId].kP, 0.001, 0.1);
    ImGui::InputDouble("kI", &mTrackedMotorConfig[motorId].kI, 0.001, 0.1);
    ImGui::InputDouble("kD", &mTrackedMotorConfig[motorId].kD, 0.001, 0.1);
    ImGui::InputDouble("kF", &mTrackedMotorConfig[motorId].kF, 0.001, 0.1);
    ImGui::InputDouble("I Zone", &mTrackedMotorConfig[motorId].iZone, 0.001, 0.1);
    ImGui::InputDouble("Max I Accum", &mTrackedMotorConfig[motorId].max_i_accum, 0.001, 0.1);
    ImGui::InputDouble("Cruise Velocity", &mTrackedMotorConfig[motorId].motion_cruise_velocity, 0.001, 0.1);
    ImGui::InputDouble("Motion Acceleration", &mTrackedMotorConfig[motorId].motion_acceleration, 0.001, 0.1);
    ImGui::InputInt("S Curve Strength", &mTrackedMotorConfig[motorId].motion_s_curve_strength, 1, 1);
    ImGui::InputDouble("Allowed CL Error", &mTrackedMotorConfig[motorId].allowed_closed_loop_error, 0.001, 0.1);
    ImGui::InputDouble("CL Peak Output", &mTrackedMotorConfig[motorId].max_closed_loop_peak_output, 0.001, 0.1);
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::InputDouble("Motor Output", &mTrackedMotorControl[motorId].output_value, 1, 1);
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if (ImGui::Button("Send Update"))
    {
        copyToLiveMotorConfig(motorId);
    }
    ImGui::End();
}

void WindowManager::copyToLiveMotorConfig(int motorId)
{
    std::scoped_lock<std::mutex> lockControl(*mMotorControlMutex);
    std::scoped_lock<std::mutex> lockConfig(*mOutputMotorConfigMutex);
    (*mMotorControlMap)[motorId].output_value = mTrackedMotorControl[motorId].output_value;
    (*mMotorConfigMap)[motorId].kP = mTrackedMotorConfig[motorId].kP;
    (*mMotorConfigMap)[motorId].kI = mTrackedMotorConfig[motorId].kI;
    (*mMotorConfigMap)[motorId].kD = mTrackedMotorConfig[motorId].kD;
    (*mMotorConfigMap)[motorId].kF = mTrackedMotorConfig[motorId].kF;
    (*mMotorConfigMap)[motorId].iZone = mTrackedMotorConfig[motorId].iZone;
    (*mMotorConfigMap)[motorId].max_i_accum = mTrackedMotorConfig[motorId].max_i_accum;
    (*mMotorConfigMap)[motorId].motion_cruise_velocity = mTrackedMotorConfig[motorId].motion_cruise_velocity;
    (*mMotorConfigMap)[motorId].motion_acceleration = mTrackedMotorConfig[motorId].motion_acceleration;
    (*mMotorConfigMap)[motorId].motion_s_curve_strength = mTrackedMotorConfig[motorId].motion_s_curve_strength;
    (*mMotorConfigMap)[motorId].allowed_closed_loop_error = mTrackedMotorConfig[motorId].allowed_closed_loop_error;
    (*mMotorConfigMap)[motorId].max_closed_loop_peak_output = mTrackedMotorConfig[motorId].max_closed_loop_peak_output;
    //ROS_INFO("Copy to Live called. Motor Id: %d, Motor output value: %f, Motor kP value: %f", motorId, (*mMotorControlMap)[motorId].output_value, (*mMotorConfigMap)[motorId].kP);
}

void WindowManager::captureTunableMotors()
{
    static ros::Publisher motor_tuning_status_pub = node->advertise<rio_control_node::Motor_Status>("MotorTuningStatus", 1);
	static rio_control_node::Motor_Status motorStatus;
    std::lock_guard<std::mutex> lockConfig(*mOutputMotorConfigMutex);
    std::lock_guard<std::mutex> lockControl(*mMotorControlMutex);
    std::lock_guard<std::mutex> lockStatus(*mMotorInfoMutex);
    for (const std::pair<uint32_t, rio_control_node::Motor_Config>& m : *mMotorConfigMap)
    {
        if (m.second.controller_mode == rio_control_node::Motor_Config::MASTER
            || m.second.controller_mode == rio_control_node::Motor_Config::FAST_MASTER)
        {
            if (!mTrackedMotorConfig.count(m.first))
            {
                mTrackedMotorConfig[m.first] = m.second;
            }
            if (!mTrackedMotorControl.count(m.first))
            {
                mTrackedMotorControl[m.first] = (*mMotorControlMap)[m.first];
            }
            mTrackedMotorStatus[m.first] = (*mMotorStatusMap)[m.first];
        }
    }

    motorStatus.motors.clear();
    for (const std::pair<uint32_t, rio_control_node::Motor_Info>& mInfo : *mMotorStatusMap)
    {
        motorStatus.motors.push_back(mInfo.second);
    }
    motor_tuning_status_pub.publish(motorStatus);
}