//
//  Timer.hpp
//  orbslam3_ios
//
//  Created by HouPeihong on 2024/3/23.
//

#ifndef Timer_hpp
#define Timer_hpp

#include <chrono>

namespace ORB_SLAM3 {
class Timer{
    std::chrono::steady_clock::time_point m_startTime;
public:
    Timer();
    void tik();
    double durationMilliSeconds();
    double durationSeconds();
    static double timeStampSeconds();
    void tok(const char* msg, bool calc_fps = false);
    void tok(const char* msg, const char* sub_msg);
};
}

#endif /* Timer_hpp */
