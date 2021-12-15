#pragma once
#include "ck_utilities/Singleton.hpp"

#include <thread>

class WindowManager : public Singleton<WindowManager>
{
    friend Singleton;
public:
    ~WindowManager();
    void showWindow();
    std::thread* getThreadHandle();
private:
    void showWindow_internal();

    std::thread* mWindowThread;
};