#ifndef USB_CAMERA_NODE_HPP
#define USB_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace rclcpp;

namespace usb_camera {
class USBCameraNode : public rclcpp::Node {
public:
    explicit USBCameraNode(const rclcpp::NodeOptions & options);
    ~USBCameraNode();

private:
    void cameraThread();
    void setCameraParameters();
    VideoCapture cap;
    Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
    std::thread camera_thread_;
    std::atomic<bool> running_;
    int width_;
    int height_;
    int fps_;
};
}

#endif