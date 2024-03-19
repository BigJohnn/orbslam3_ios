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

#define GRAVITY ((double)9.805)

@implementation ImageProcessor

using namespace Eigen;
using namespace std;
struct IMU_MSG {
    NSTimeInterval header;
    Vector3d acc;
    Vector3d gyr;
};

struct IMG_MSG {
    NSTimeInterval header;
    map<int, Vector3d> point_clouds;
};

struct IMG_DATA {
    NSTimeInterval header;
    UIImage *image;
};
typedef shared_ptr <IMU_MSG const > ImuConstPtr;
typedef shared_ptr <IMG_MSG const > ImgConstPtr;


ImageProcessor* mImageProcessor;
queue<IMG_DATA> imgDataBuf;
NSMutableData *imuDataBuf = [[NSMutableData alloc] init];
NSData *imuReader;
IMG_DATA imgData;
IMU_MSG imuData;
// Lock the feature and imu data buffer
std::mutex m_buf;
std::condition_variable con;

int imu_prepare = 0;

NSTimeInterval current_time = -1;

NSTimeInterval lateast_imu_time = -1;



// Store the feature data
queue<ImgConstPtr> img_msg_buf;

// Store the IMU data
queue<ImuConstPtr> imu_msg_buf;
// Lock the IMU data feedback to featuretracker
std::mutex m_imu_feedback;

+ (ImageProcessor *) shared {
    if(!mImageProcessor) {
        mImageProcessor = [ImageProcessor new];
        [mImageProcessor imuStartUpdate];
        mImageProcessor.motionManager = [[CMMotionManager alloc] init];
    }
    return mImageProcessor;
}

- (nonnull UIImage *)Process:(nonnull UIImage *)img {
    cv::Mat src;
    using namespace cv;
        UIImageToMat(img, src);
    
//        resizeImg(src);
//        cv::medianBlur(src, src, 3);
//        cv::Mat sharpen_op = (cv::Mat_<char>(3, 3) <<
//                              0, -1, 0,
//                              -1, 5, -1,
//                              0, -1, 0);
//        filter2D(src, src, CV_32F, sharpen_op);
//        convertScaleAbs(src, src);
        cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);

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
shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());
vector<IMU_MSG> gyro_buf;  // for Interpolation
- (void)imuStartUpdate
{
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
         motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch for vins
         motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll for vins
         if(imu_prepare<10)
         {
             imu_prepare++;
         }
         shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
         acc_msg->header = latestAcc.timestamp;
         acc_msg->acc << -latestAcc.acceleration.x * GRAVITY,
         -latestAcc.acceleration.y * GRAVITY,
         -latestAcc.acceleration.z * GRAVITY;
         cur_acc = acc_msg;
         //printf("imu acc update %lf %lf %lf %lf\n", acc_msg->header, acc_msg->acc.x(), acc_msg->acc.y(), acc_msg->acc.z());
         
     }];
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
         //The time stamp is the amount of time in seconds since the device booted.
         NSTimeInterval header = latestGyro.timestamp;
         if(header<=0)
             return;
         if(imu_prepare < 10)
             return;
         
         IMU_MSG gyro_msg;
         gyro_msg.header = header;
         gyro_msg.gyr << latestGyro.rotationRate.x,
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
         shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
         if(cur_acc->header >= gyro_buf[0].header && cur_acc->header < gyro_buf[1].header)
         {
             imu_msg->header = cur_acc->header;
             imu_msg->acc = cur_acc->acc;
             imu_msg->gyr = gyro_buf[0].gyr + (cur_acc->header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
             //printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].header, imu_msg->header, gyro_buf[1].header);
             //printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, gyro_buf[0].gyr.x(), imu_msg->gyr.x(), gyro_buf[1].gyr.x());
         }
         else
         {
             printf("imu error %lf %lf %lf\n", gyro_buf[0].header, cur_acc->header, gyro_buf[1].header);
             return;
         }
         
         //for save data
//         if(start_playback)
//         {
//             //TS(read_imu_buf);
//             if(imuDataFinished)
//                 return;
//             [imuReader getBytes:&imuData range: NSMakeRange(imuDataReadIndex * sizeof(imuData), sizeof(imuData))];
//             imuDataReadIndex++;
//             if(imuData.header == 0)
//             {
//                 imuDataFinished = true;
//                 return;
//             }
//             imu_msg->header = imuData.header;
//             imu_msg->acc = imuData.acc;
//             imu_msg->gyr = imuData.gyr;
//             //TE(read_imu_buf);
//         }
         
//         if(start_record)
//         {
//             TS(record_imu_buf);
//             imuData.header = imu_msg->header;
//             imuData.acc = imu_msg->acc;
//             imuData.gyr = imu_msg->gyr;
//             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
//             imuDataIndex++;
//             TE(record_imu_buf);
////             NSLog(@"record: imu %lf, %lu",imuData.header,imuDataIndex);
//         }
         
         lateast_imu_time = imu_msg->header;
         
         //img_msg callback
//         {
//             IMU_MSG_LOCAL imu_msg_local;
//             imu_msg_local.header = imu_msg->header;
//             imu_msg_local.acc = imu_msg->acc;
//             imu_msg_local.gyr = imu_msg->gyr;
//             
//             m_imu_feedback.lock();
//             local_imu_msg_buf.push(imu_msg_local);
//             m_imu_feedback.unlock();
//         }
         m_buf.lock();
         imu_msg_buf.push(imu_msg);
         NSLog(@"IMU_buf timestamp %lf, acc_x = %lf",imu_msg_buf.front()->header,imu_msg_buf.front()->acc.x());
         m_buf.unlock();
         con.notify_one();
     }];
}


@end
