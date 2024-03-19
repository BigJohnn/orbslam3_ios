//
//  CameraController.swift
//
//  Created by John Smith on 2021/8/20.
//

import Foundation
import AVFoundation
import UIKit

@Observable class CameraController: NSObject,  AVCaptureVideoDataOutputSampleBufferDelegate {
    var session = AVCaptureSession()
    var output = AVCaptureVideoDataOutput();

    var input: AVCaptureDeviceInput!
    var preview: AVCaptureVideoPreviewLayer!
    var processedImg: UIImage?
    
    let imageProc = ImageProcessor.shared()
    private let processingSemaphore = DispatchSemaphore(value: 1)
    private let videoDataOutputQueue = DispatchQueue(label: "VideoDataOutput", qos: .userInitiated, attributes: [], autoreleaseFrequency: .workItem)
//    private var lock = false
    
    func check() {
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized: // 已被用户同意使用摄像头
            self.setup()
            return;
        case .notDetermined: // 首次请求使用摄像头
            AVCaptureDevice.requestAccess(for: .video) {[weak self] granted in
                if granted {
                    self?.setup()
                }
            }
            return;
        case .denied: // 用户拒绝了摄像头调用申请
            return
            
        case .restricted: // 用户无法开启摄像头
            return
        @unknown default:
            return;
        }
    }
    
    func setup() {
        do {
            session.beginConfiguration()
            let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back);
            input = try AVCaptureDeviceInput(device: device!)
            if session.canAddInput(input) {
                session.addInput(input)
            }
            if session.canAddOutput(output) {
                session.addOutput(output)
            }
            output.setSampleBufferDelegate(self, queue: videoDataOutputQueue)
            session.commitConfiguration()
        } catch {
            print(error.localizedDescription)
        }
    }
    
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {

        guard processingSemaphore.wait(timeout: .now()) == .success,
                      let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer)
                else {
                    return
                }

        let ciImage = CIImage(cvPixelBuffer: pixelBuffer).oriented(.right)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else {
            print("错误：无法获取图像！")
//            lock = false;
            return
        }
        let image = UIImage(cgImage: cgImage)
        DispatchQueue.global().async {
            self.processedImg = self.imageProc.process(image.fixOrientation())
            self.processingSemaphore.signal()

         
        }
    }
}
