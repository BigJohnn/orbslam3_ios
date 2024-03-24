//
//  image_processor.cpp
//  orbslam3_ios
//
//  Created by HouPeihong on 2024/3/19.
//
#import <opencv2/opencv.hpp>
#import <opencv2/imgcodecs/ios.h>


#import "image_processor.h"
#include <Eigen/Dense>
#include <System.h>

#define GRAVITY ((double)9.805)

@implementation ImageProcessor

using namespace Eigen;
using namespace std;

using namespace ORB_SLAM3;
std::mutex mutexImuMeas;
vector<IMU::Point> vImuMeas;

ImageProcessor* mImageProcessor;

NSMutableData *imuDataBuf = [[NSMutableData alloc] init];
NSData *imuReader;
//IMU_MSG imuData;
// Lock the feature and imu data buffer
//std::mutex m_buf;
std::condition_variable con;

int imu_prepare = 0;

NSTimeInterval current_time = -1;

NSTimeInterval lateast_imu_time = -1;

// Store the IMU data
queue<IMU::Point> meas;
// Lock the IMU data feedback to featuretracker
std::mutex m_imu_feedback;


std::shared_ptr<System> slam;
std::thread track_thread;
std::mutex track_mutex;


+ (ImageProcessor *) shared {
    if(!mImageProcessor) {
        mImageProcessor = [ImageProcessor new];
        [mImageProcessor imuStartUpdate];
        slam.reset();
        
        dispatch_async(dispatch_get_global_queue( DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^(void){
            auto* voc_path = [[[NSBundle mainBundle] pathForResource:@"ORBvoc" ofType:@"bin"] UTF8String];
            slam = std::make_shared<System>(voc_path,
                                            [[[NSBundle mainBundle] pathForResource:@"ipxsmax_test" ofType:@"yaml"] UTF8String], System::IMU_MONOCULAR);
        });
        
    }
    return mImageProcessor;
}

bool is_tracking = false;
bool is_imu_started = false;
- (nonnull UIImage *)Process:(nonnull UIImage *)img {
    if(!is_tracking) {
        static auto track_func = []() {
            
            while(!slam) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            int t_prev = 0;
            
            while(true) {
                
                Timer timer;
                
                double t = 0;
                
                auto && cf = slam->getCFScaled(t);
                
                vector<IMU::Point> meas_cur_frame;
                if(!cf.empty()) {
                    {
                        if(0 == t_prev) {
                            t_prev = t;
                        }
                        
                        std::unique_lock<std::mutex> lk(mutexImuMeas);
                        
                        if(!meas.empty()) {
                            while(!meas.empty() && meas.front().t <= t) {
                                meas_cur_frame.push_back(meas.front());
//                                printf("meas_cur_frame.push_back(%lf)\n", meas.front().t);
                                meas.pop();
                            }
                        }
                    }
                    
                    // Only track if there's both image and IMU data
                    if (!cf.empty() && !meas_cur_frame.empty()) {
                        const int capacity = 10;
                        while(meas_cur_frame.size()>capacity) meas_cur_frame.erase(meas_cur_frame.begin());
                        string tname = std::to_string(t);
                        printf("meas_cur_frame.size == %lu", meas_cur_frame.size());
                        
                        {
                            slam->TrackMonocular(cf, t, meas_cur_frame, tname);
                        }
                        
                        timer.tok("Track spent", true);
                        auto t2 = timer.durationMilliSeconds();
                        float track_fps = std::min(30.0f, static_cast<float>(1000/t2));

                        auto dt = (t - t_prev) * 1e3;
                        if(t2 < 33) {
                            usleep((33-t2) * 1e3);
                        }
                    }
                }

            }
        };
        track_thread = thread(track_func);
        track_thread.detach();
        
        is_tracking = true;
    }
    
    cv::Mat src;
    using namespace cv;
    UIImageToMat(img, src);
    

    {
        if(slam) {
            
            slam->setCFScaled(src, [[NSProcessInfo processInfo] systemUptime]);
        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    
    
    
    return MatToUIImage(src);
}

/*
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 IMU data process and interploration
 
 */
bool imuDataFinished = false;
bool vinsDataFinished = false;
shared_ptr<IMU::Point> cur_acc(new IMU::Point(0,0,0,0,0,0,0));
vector<IMU::Point> gyro_buf;  // for Interpolation
- (void)imuStartUpdate
{
    printf("imuStartUpdate\n ");
    CMMotionManager *motionManager = [[CMMotionManager alloc] init];
    if (!motionManager.accelerometerAvailable) {
        NSLog(@"没有加速计");
    }
#ifdef DATA_EXPORT
    motionManager.accelerometerUpdateInterval = 0.1;
    motionManager.gyroUpdateInterval = 0.1;
#else
    motionManager.accelerometerUpdateInterval = 0.01;
    motionManager.gyroUpdateInterval = 0.01;
#endif
    
    [motionManager startDeviceMotionUpdates];
    
    [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                        withHandler:^(CMAccelerometerData *latestAcc, NSError *error)
     {
        
         double header = motionManager.deviceMotion.timestamp;
         motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch
         motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll
         if(imu_prepare<10)
         {
             imu_prepare++;
         }
        
         shared_ptr<IMU::Point> acc_msg(new IMU::Point(0,0,0,0,0,0,0));
         acc_msg->t = latestAcc.timestamp;
//         acc_msg->a << -latestAcc.acceleration.x * GRAVITY,
//         -latestAcc.acceleration.y * GRAVITY,
//         -latestAcc.acceleration.z * GRAVITY;
        acc_msg->a << latestAcc.acceleration.x,
        -latestAcc.acceleration.y,
        latestAcc.acceleration.z;
         cur_acc = acc_msg;
//         printf("imu acc update %lf %lf %lf %lf\n", acc_msg->t, acc_msg->a.x(), acc_msg->a.y(), acc_msg->a.z());
         
     }];
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
        Timer timer;
        timer.tik();
         //The time stamp is the amount of time in seconds since the device booted.
         NSTimeInterval header = latestGyro.timestamp;
         if(header<=0)
             return;
         if(imu_prepare < 10)
             return;
         
        IMU::Point gyro_msg;
         gyro_msg.t = header;
         gyro_msg.w << latestGyro.rotationRate.x,
         -latestGyro.rotationRate.y,
         latestGyro.rotationRate.z;
         
         if(gyro_buf.size() == 0)
         {
             gyro_buf.push_back(gyro_msg);
             gyro_buf.push_back(gyro_msg);
             return;
         }
         else
         {
             gyro_buf[0] = gyro_buf[1];
             gyro_buf[1] = gyro_msg;
         }
         //interpolation
         IMU::Point imu_msg;
         if(cur_acc->t >= gyro_buf[0].t && cur_acc->t < gyro_buf[1].t)
         {
             imu_msg.t = cur_acc->t;
             imu_msg.a = cur_acc->a;
             
             // w 0  1
             // a 0  1
             imu_msg.w = gyro_buf[0].w + (cur_acc->t - gyro_buf[0].t)*(gyro_buf[1].w - gyro_buf[0].w)/(gyro_buf[1].t - gyro_buf[0].t);
//             printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].t, imu_msg->t, gyro_buf[1].t);
//             printf("imu inte update %lf %lf %lf %lf %lf %lf %lf\n", imu_msg.t,
//                    imu_msg.a.x(), imu_msg.a.y(), imu_msg.a.z(),
//                    imu_msg.w.x(), imu_msg.w.y(), imu_msg.w.z()
//                    );
         }
         else
         {
             printf("imu error %lf %lf %lf\n", gyro_buf[0].t, cur_acc->t, gyro_buf[1].t);
             return;
         }
         
        {
            std::unique_lock<std::mutex> lock2(mutexImuMeas);
             meas.push(imu_msg);
        }
     }];
}

@end
