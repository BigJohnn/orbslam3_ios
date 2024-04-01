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

int imu_prepare = 0;

NSTimeInterval current_time = -1;

NSTimeInterval lateast_imu_time = -1;

// Store the IMU data
queue<IMU::Point> meas;

std::shared_ptr<System> slam;
std::thread track_thread;
std::mutex track_mutex;

std::condition_variable con;

cv::String tmp_folder;

//#define CALIB_MODE

#define S2NS 1e9

struct ImageCache{
    cv::Mat image;
    double timestamp;
};
std::queue<ImageCache> cached_imgs;

+ (instancetype)shared {
  static ImageProcessor *sharedInstance = nil;
  static dispatch_once_t onceToken;
  dispatch_once(&onceToken, ^{
    sharedInstance = [[ImageProcessor alloc] init];
  });
    [sharedInstance initialize];
  return sharedInstance;
}

- (void)initialize {
  // Thread-safe initialization of instance variables (if needed)
  
    [self imuStartUpdate];
#ifndef CALIB_MODE
        dispatch_async(dispatch_get_global_queue( DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^(void){
            auto* voc_path = [[[NSBundle mainBundle] pathForResource:@"ORBvoc" ofType:@"bin"] UTF8String];
            slam = std::make_shared<System>(voc_path,
                                            [[[NSBundle mainBundle] pathForResource:@"ipxsmax_test" ofType:@"yaml"] UTF8String], System::IMU_MONOCULAR);
        });
#else
        NSString* tmppath = NSFileManager.defaultManager.temporaryDirectory.path;
        tmp_folder = cv::String([tmppath cStringUsingEncoding:kCFStringEncodingUTF8])+cv::String("/");
        std::ofstream outFile(tmp_folder + "times.txt", std::ios::out);
        outFile.close();
        std::ofstream accOutFile(tmp_folder + "acc.txt", std::ios::out);
        accOutFile.close();
        std::ofstream gyrOutFile(tmp_folder + "gyr.txt", std::ios::out);
        gyrOutFile.close();
        
#endif

}

std::vector<std::pair<std::vector<IMU::Point>, ImageCache>>
getMeasurements()
{
    std::vector<std::pair<std::vector<IMU::Point>, ImageCache>> measurements;
    while (true)
    {
        auto&& cache = cached_imgs;
        
        vector<IMU::Point> imus;
        
        if(cache.empty() || meas.empty()) {
            return measurements;
        }
        if (!(meas.back().t > cache.front().timestamp))
        {
            NSLog(@"wait for imu, only should happen at the beginning");
            return measurements;
        }
        if (!(meas.front().t < cache.front().timestamp))
        {
            NSLog(@"throw img, only should happen at the beginning");
            cache.pop();
            continue;
        }
        ImageCache curImg = cache.front();
        cache.pop();
        
        printf("cache size == %lu\n", cache.size());
        
        while (meas.front().t <= curImg.timestamp)
        {
            imus.emplace_back(meas.front());
            meas.pop();
        }
        measurements.emplace_back(imus, curImg);
    }
    return measurements;
}

bool is_tracking = false;
bool is_imu_started = false;
- (nonnull UIImage *)Process:(nonnull UIImage *)img {
#ifndef CALIB_MODE
    if(!is_tracking) {
        static auto track_func = []() {
            
            while(!slam) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            while(true) {
                
                Timer timer;
                
                std::vector<std::pair<std::vector<IMU::Point>, ImageCache>> measurements;
                std::unique_lock<std::mutex> lk(mutexImuMeas);
                con.wait(lk, [&]
                         {
                             return (measurements = getMeasurements()).size() != 0;
                         });
                lk.unlock();
                
                for(auto&& meas: measurements) {
                    auto&& curImg = meas.second;
                    string tname = std::to_string(curImg.timestamp);
//                    printf("track with %lu measures!\n", meas.first.size());
                    slam->TrackMonocular(curImg.image, curImg.timestamp, meas.first, tname);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        };
        track_thread = thread(track_func);
        track_thread.detach();
        
        is_tracking = true;
    }
#else
    std::ofstream outFile(tmp_folder + "times.txt", std::ios::app);
    auto&& t = [[NSProcessInfo processInfo] systemUptime];
    outFile << std::to_string(t) << std::endl;
    cout << "[img]"<< std::to_string(t) <<endl;
    outFile.close();
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    static int curId=0;
    int period = 90;
    if(curId % period != 0) {
        curId = (++curId) % period;
        return img;
    }
#endif
    
    Timer render_tm;
    cv::Mat src;
    using namespace cv;
    UIImageToMat(img, src);
    
    {
#ifndef CALIB_MODE
        if(slam) {
//            static int curId = 0;
//            if((curId = curId++ % 5) == 0) {
            cv::Mat gray;
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
            float scale = 0.5f;
            cv::resize(gray, gray, cv::Size(int(src.cols * scale), int(src.rows * scale)));
            
            mutexImuMeas.lock();
//            slam->setCFScaled(gray, [[NSProcessInfo processInfo] systemUptime]);
            cached_imgs.push(ImageCache{gray, [[NSProcessInfo processInfo] systemUptime]});
            mutexImuMeas.unlock();
            con.notify_one();
//            }
        }
#else
    
        cv::Mat gray, grayScaled;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        float scale = 0.5f;
        cv::resize(gray, grayScaled, cv::Size(int(gray.cols * scale), int(gray.rows * scale)));
        
        static int i = 0;
        ++i;
        cv::imwrite(tmp_folder + std::to_string(long(t*1e9)) + ".png", grayScaled);
        
        if(i >= 300) {
            exit(0);
        }
#endif
        
    }
    
    
    
//    render_tm.tok("draw single frame spent");
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
        /*此处不加会引起传感器数据不更新， 疑似bug*/
        double header = motionManager.deviceMotion.timestamp;
        motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch
        motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll
        
#ifndef CALIB_MODE
         
         if(imu_prepare<10)
         {
             imu_prepare++;
         }
        
         shared_ptr<IMU::Point> acc_msg(new IMU::Point(0,0,0,0,0,0,0));
         acc_msg->t = latestAcc.timestamp;
//         acc_msg->a << -latestAcc.acceleration.x * GRAVITY,
//         -latestAcc.acceleration.y * GRAVITY,
//         -latestAcc.acceleration.z * GRAVITY;
        acc_msg->a << -latestAcc.acceleration.x,
        -latestAcc.acceleration.y,
        -latestAcc.acceleration.z;
         cur_acc = acc_msg;
#else
        std::ofstream accOutFile(tmp_folder + "acc.txt", std::ios::app);
        auto&& t = (latestAcc.timestamp);
        accOutFile << std::to_string(t) <<","
        << std::to_string(latestAcc.acceleration.x) <<","
        << std::to_string(latestAcc.acceleration.y) <<","
        << std::to_string(latestAcc.acceleration.z) << std::endl;
        cout<< "[acc]" <<std::to_string(t) <<endl;
        accOutFile.close();
#endif
//         printf("imu acc update %lf %lf %lf %lf\n", acc_msg->t, acc_msg->a.x(), acc_msg->a.y(), acc_msg->a.z());
         
     }];
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
#ifndef CALIB_MODE
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
         latestGyro.rotationRate.y,
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
         if(cur_acc->t >= gyro_buf[0].t && cur_acc->t <= gyro_buf[1].t)
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
         
        
        mutexImuMeas.lock();
        meas.push(imu_msg);
        mutexImuMeas.unlock();
        con.notify_one();
        
#else
        std::ofstream gyroOutFile(tmp_folder + "gyr.txt", std::ios::app);
        auto&& t = (latestGyro.timestamp);
        gyroOutFile << std::to_string(t) <<","
        << std::to_string(latestGyro.rotationRate.x) <<","
        << std::to_string(latestGyro.rotationRate.y) <<","
        << std::to_string(latestGyro.rotationRate.z) << std::endl;
        printf("[gyr]%s\n", std::to_string(t).c_str());
//        cout<< "[gyr]"<< std::to_string(latestGyro.timestamp * S2NS) <<endl;
        gyroOutFile.close();
#endif
     }];
}

@end
