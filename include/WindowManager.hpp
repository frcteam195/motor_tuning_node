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
private:
    void showWindow_internal();

    bool mUpdateRequested;
    std::mutex mUpdateMutex;

    std::thread* mWindowThread;
    std::map<uint32_t, rio_control_node::Motor_Config>* mMotorConfigMap;
    std::map<uint32_t, rio_control_node::Motor_Info>* mMotorStatusMap;
    std::mutex* mOutputMotorConfigMutex;
    std::mutex* mMotorInfoMutex;

};