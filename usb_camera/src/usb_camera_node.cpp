#include "usb_camera/usb_camera_node.hpp"

namespace usb_camera {
    USBCameraNode::USBCameraNode(const rclcpp::NodeOptions & options)
    : Node("usb_camera", options), running_(true) {
    // 声明参数
    declare_parameter("device_path", "/dev/video0");
    declare_parameter("frame_id", "camera");
    declare_parameter("topic_name", "image_raw");
    declare_parameter("width", 1920);
    declare_parameter("height", 1080);
    declare_parameter("fps", 30);
    
    // 获取参数
    auto device_path = get_parameter("device_path").as_string();
    auto frame_id = get_parameter("frame_id").as_string();
    auto topic_name = get_parameter("topic_name").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_int();

    // 初始化摄像头
    cap.open(device_path, cv::CAP_V4L2);
    if(!cap.isOpened()){
      RCLCPP_ERROR(get_logger(), "Failed to open camera device: %s", device_path.c_str());
      throw std::runtime_error("Camera open failed");
    }

    // 设置相机参数
    setCameraParameters();

    // 创建图像发布者
    camera_pub_ = create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

    // 启动相机线程
    camera_thread_ = std::thread(&USBCameraNode::cameraThread, this);
  }

  void USBCameraNode::setCameraParameters() {
    // 首先尝试设置MJPG格式
    if(!cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'))) {
      RCLCPP_WARN(get_logger(), "Failed to set MJPG format, trying YUYV");
      if(!cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'))) {
        RCLCPP_ERROR(get_logger(), "Failed to set any supported format");
      }
    }

    // 设置分辨率
    if(!cap.set(cv::CAP_PROP_FRAME_WIDTH, width_) || 
       !cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_)) {
      RCLCPP_WARN(get_logger(), "Failed to set resolution %dx%d, trying lower resolution", width_, height_);

      cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }

    // 设置帧率
    if(!cap.set(cv::CAP_PROP_FPS, fps_)) {
      RCLCPP_WARN(get_logger(), "Failed to set FPS to %d", fps_);
    }

    // 设置缓冲区大小
    cap.set(cv::CAP_PROP_BUFFERSIZE, 3);

    // 禁用自动曝光和自动白平衡
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); // 0.25表示手动曝光
    cap.set(cv::CAP_PROP_AUTO_WB, 0);

    // 验证设置是否成功
    int actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int actual_fps = cap.get(cv::CAP_PROP_FPS);
    int actual_fourcc = cap.get(cv::CAP_PROP_FOURCC);

    RCLCPP_INFO(get_logger(), "Camera parameters set: %dx%d @ %d fps, format: %c%c%c%c", 
        actual_width, actual_height, actual_fps,
        (actual_fourcc & 0xFF),
        ((actual_fourcc >> 8) & 0xFF),
        ((actual_fourcc >> 16) & 0xFF),
        ((actual_fourcc >> 24) & 0xFF));
  }

  USBCameraNode::~USBCameraNode() {
    running_ = false;
    if (camera_thread_.joinable()) {
      camera_thread_.join();
    }
  }

  void USBCameraNode::cameraThread() {
    auto frame_id = get_parameter("frame_id").as_string();
    cv::Mat frame;
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    
    while (running_) {
      auto current_time = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
      //RCLCPP_INFO(get_logger(), "Elapsed time: %ld ms", elapsed);
      if(elapsed >= 1000) {
        RCLCPP_INFO(get_logger(), "Current FPS: %d", frame_count);
        frame_count = 0;
        last_time = current_time;
      }
      
      if(cap.grab()) {
        if(cap.retrieve(frame)) {
          auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame
          ).toImageMsg();
          
          msg->header.stamp = now();
          msg->header.frame_id = frame_id;
          camera_pub_->publish(*msg);
          frame_count++;
        }
      }
      
      // 动态调整延迟
      auto process_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - current_time).count();
      int sleep_time = std::max(1, (1000/fps_) - static_cast<int>(process_time));
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera::USBCameraNode)