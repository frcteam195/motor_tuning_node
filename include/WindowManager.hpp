#pragma once
#include "ck_utilities/Singleton.hpp"

#include <mutex>
#include <thread>
#include <map>
#include <rio_control_node/Motor_Config.h>
#include <rio_control_node/Motor_Info.h>


class WindowManager : public Singleton<WindowManager>
{
    friend Singleton;
public:
    ~WindowManager();
    void showWindow(std::map<uint32_t, rio_control_node::Motor_Config>& currMotorConfigMap, std::mutex& motorConfigMutex, std::map<uint32_t, rio_control_node::Motor_Info>& currMotorStatusMap, std::mutex& motorInfoMutex);
    std::thread* getThreadHandle();
    bool isUpdateRequested();
    void resetUpdateRequested();
    void drawImguiWindow(int motorId, int listPos);
private:
    void showWindowOpenGL2_internal();
    void showWindowOpenGL3_internal();
    void captureTunableMotors();

    bool mUpdateRequested;
    std::mutex mUpdateMutex;

    std::thread* mWindowThread;
    std::map<uint32_t, rio_control_node::Motor_Config>* mMotorConfigMap;
    std::map<uint32_t, rio_control_node::Motor_Info>* mMotorStatusMap;
    std::mutex* mOutputMotorConfigMutex;
    std::mutex* mMotorInfoMutex;

    std::map<uint32_t, rio_control_node::Motor_Config> mTrackedMotors;

};