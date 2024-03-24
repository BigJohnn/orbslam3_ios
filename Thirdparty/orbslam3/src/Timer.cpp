//
//  Timer.cpp
//  orbslam3_ios
//
//  Created by HouPeihong on 2024/3/23.
//

#include "Timer.h"
#include <cstdio>
#include <string>
using namespace std::chrono;

namespace ORB_SLAM3 {
Timer::Timer() {
    tik();
}


void Timer::tik()
{
    m_startTime = steady_clock::now();
}
double Timer::durationMilliSeconds()
{
    auto&&t = steady_clock::now();
    return duration_cast<duration<double>>(t -m_startTime).count() * 1000.0;
}
double Timer::durationSeconds()
{
    auto&&t = steady_clock::now();
    return duration_cast<duration<double>>(t -m_startTime).count();
}

double Timer::timeStampSeconds()
{
    auto now = std::chrono::steady_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = now_ns.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
    return value.count()*1.0e-9;
}
void Timer::tok(const char* msg, bool calc_fps)
{
    auto t = steady_clock::now();
    auto dur = duration_cast<duration<double>>(t-m_startTime).count();
    char x[200];
    if(calc_fps) {
        sprintf(x, "%s spent %f ms! (%f FPS)\n", msg, dur *1000.0,1.0/dur);
    }
    else {
        sprintf(x, "%s spent %f ms!\n", msg, dur *1000.0);
    }
    
    printf("%s", x);
}

void Timer::tok(const char* msg, const char* sub_msg) {
    tok((std::string(msg)+std::string(sub_msg)).c_str());
}
}
