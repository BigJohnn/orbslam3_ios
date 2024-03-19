//
//  image_processor.hpp
//  orbslam3_ios
//
//  Created by HouPeihong on 2024/3/19.
//

#ifndef image_processor_hpp
#define image_processor_hpp

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>

NS_ASSUME_NONNULL_BEGIN


@interface ImageProcessor : NSObject

@property (nonatomic, strong) CMMotionManager *motionManager;

+ (ImageProcessor *) shared;
- (UIImage *) Process: (UIImage *) img;
- (void)imuStartUpdate;


@end

NS_ASSUME_NONNULL_END

#endif /* image_processor_hpp */
