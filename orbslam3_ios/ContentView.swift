//
//  ContentView.swift
//  orbslam3_ios
//
//  Created by HouPeihong on 2024/3/19.
//

import SwiftUI

struct ProcessResultView: UIViewRepresentable {
  var img: UIImage

  func makeUIView(context: Context) -> UIView {
      let view = UIView(frame: UIScreen.main.bounds)
      view.backgroundColor = .red

      let imageView = UIImageView(image: img)

      imageView.translatesAutoresizingMaskIntoConstraints = false // Enable auto layout constraints

      view.addSubview(imageView) // Add imageView as a subview

      // Add constraints to position the imageView within the view
      NSLayoutConstraint.activate([
        imageView.topAnchor.constraint(equalTo: view.topAnchor),
        imageView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
        imageView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
        imageView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
      ])
      return view
  }

  func updateUIView(_ uiView: UIViewType, context: Context) {
      guard let imageView = uiView.subviews.first as? UIImageView else { return }
          imageView.image = img // Update image when needed
  }
}

struct ContentView: View {
     private var cameraController = CameraController.init()
    var body: some View {
        ZStack {
            
            if(cameraController.processedImg != nil) {
                ProcessResultView(img: cameraController.processedImg!)
            }
            else {
                CameraPreviewView(cameraController: cameraController)
            }
        }
        .padding()
    }
}

#Preview {
    ContentView()
}
