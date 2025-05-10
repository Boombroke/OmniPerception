// Copyright 2022 Chen Jun
// Copyright 2025 Boombroke
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_omni_perception/armor.hpp"
#include "rm_omni_perception/omni_perception_node.hpp"

namespace rm_omni_perception{
OmniPerceptionNode::OmniPerceptionNode(const rclcpp::NodeOptions & options)
: Node("omni_perception_node",options)
{
    RCLCPP_INFO(this->get_logger(), "Starting OmniPerceptionNode!");
    
    // 初始化检测器
    detector_ = initDetector();
    if (!detector_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize detector!");
        return;
    }
    
    // Armors Publisher
    // armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    // "/omniperception/armors", rclcpp::SensorDataQoS());   //用于发布检测到的装甲目标信息。

    // Omni Publisher //用于发布检测到的装甲目标信息。
    omni_left_pub_ = this->create_publisher<auto_aim_interfaces::msg::Omni>(
    "/omniperception/omni_left", rclcpp::SensorDataQoS());
    omni_mid_pub_ = this->create_publisher<auto_aim_interfaces::msg::Omni>(
    "/omniperception/omni_mid", rclcpp::SensorDataQoS());
    omni_right_pub_ = this->create_publisher<auto_aim_interfaces::msg::Omni>(
    "/omniperception/omni_right", rclcpp::SensorDataQoS());

    // Image Subscriber
    img_sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_left/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&OmniPerceptionNode::imageCallbackLeft, this, std::placeholders::_1));
    img_sub_mid_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_mid/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&OmniPerceptionNode::imageCallbackMid, this, std::placeholders::_1));
    img_sub_right_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_right/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&OmniPerceptionNode::imageCallbackRight, this, std::placeholders::_1));

    // 初始化debug参数
    debug_ = this->declare_parameter("debug", true);
    // 创建调试发布者
    if (debug_) {
        createDebugPublishers();
    }
    
    RCLCPP_INFO(this->get_logger(), "OmniPerceptionNode initialized successfully!");
}

void OmniPerceptionNode::imageCallbackLeft(const sensor_msgs::msg::Image::SharedPtr img_msg){
    auto armors = detectArmors(img_msg);
    auto omni_msg =auto_aim_interfaces::msg::Omni();
    if(!armors.empty()){
        for(const auto & armor : armors){
            omni_msg.detectleft =std::atoi(armor.number.c_str());
            omni_left_pub_->publish(omni_msg);
        }
    }
    omni_msg.detectleft =0;
    omni_left_pub_->publish(omni_msg);
}

void OmniPerceptionNode::imageCallbackMid(const sensor_msgs::msg::Image::SharedPtr img_msg){
    auto armors = detectArmors(img_msg);
    auto omni_msg =auto_aim_interfaces::msg::Omni();
    if(!armors.empty()){
        for(const auto & armor : armors){
            omni_msg.detectmid =std::atoi(armor.number.c_str());
            omni_mid_pub_->publish(omni_msg);
        }
    }
    omni_msg.detectmid =0;
    omni_mid_pub_->publish(omni_msg);
}

void OmniPerceptionNode::imageCallbackRight(const sensor_msgs::msg::Image::SharedPtr img_msg){
    auto armors = detectArmors(img_msg);
    auto omni_msg =auto_aim_interfaces::msg::Omni();
    if(!armors.empty()){
        for(const auto & armor : armors){
            omni_msg.detectright =std::atoi(armor.number.c_str());
            omni_right_pub_->publish(omni_msg);
        }
    }
    omni_msg.detectright =0;
    omni_right_pub_->publish(omni_msg);
}

std::vector<Armor> OmniPerceptionNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg){
    // Convert ROS img to cv::Mat
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    // Update params
    detector_->binary_thres = get_parameter("binary_thres").as_int();
    detector_->detect_color = get_parameter("detect_color").as_int();
    detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

    auto armors=detector_->detect(img);

    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_INFO(get_logger(), "Latency: %lf ms",latency);

    // Publish debug info
    if (debug_) {
        //binary_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
        detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
        [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
        detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
        [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
        auto all_num_img = detector_->getAllNumbersImage();
        //number_img_pub_.publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    detector_->drawResults(img);
    // Draw camera center
    //cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    //result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }

    return armors;
}

std::unique_ptr<Detector> OmniPerceptionNode::initDetector(){
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = declare_parameter("binary_thres", 160, param_desc);

    param_desc.description = "0-RED, 1-BLUE";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    auto detect_color = declare_parameter("detect_color", RED, param_desc);

    Detector::LightParams l_params = {
        declare_parameter("light.min_ratio", 0.1),
        declare_parameter("light.max_ratio", 0.4),
        declare_parameter("light.max_angle", 40.0)
    };

    Detector::ArmorParams a_params = {
        declare_parameter("armor.min_light_ratio", 0.7),
        declare_parameter("armor.min_small_center_distance", 0.8),
        declare_parameter("armor.max_small_center_distance", 3.2),
        declare_parameter("armor.min_large_center_distance", 3.2),
        declare_parameter("armor.max_large_center_distance", 5.5),
        declare_parameter("armor.max_angle", 35.0)
    };

    auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);//使用上述设置的参数，通过 std::make_unique<Detector> 创建了一个装甲板检测器对象 detector。
  
    // Init classifier
    auto pkg_path = ament_index_cpp::get_package_share_directory("rm_omni_perception");// 获取了装甲板检测器包的路径，并构建了模型和标签文件的完整路径。
    auto model_path = pkg_path + "/model/mlp.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    double threshold = this->declare_parameter("classifier_threshold", 0.7);
    std::vector<std::string> ignore_classes =this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
    detector->classifier =     //使用这些路径和其他参数，创建了一个 NumberClassifier 对象，并将其赋值给 detector->classifier，用于数字分类。
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
}

void OmniPerceptionNode::createDebugPublishers(){
    lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/OmniPerception/debug_lights", 10);
    armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/OmniPerception/debug_armors", 10);

    binary_img_pub_ = image_transport::create_publisher(this, "/OmniPerception/binary_img");
    number_img_pub_ = image_transport::create_publisher(this, "/OmniPerception/number_img");
    result_img_pub_ = image_transport::create_publisher(this, "/OmniPerception/result_img");
}

void OmniPerceptionNode::destroyDebugPublishers(){
    lights_data_pub_.reset();
    armors_data_pub_.reset();
    binary_img_pub_.shutdown();
    number_img_pub_.shutdown();
    result_img_pub_.shutdown();
}

void OmniPerceptionNode::publishMarkers(){
    using Marker = visualization_msgs::msg::Marker;
    armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
}


}//namespace rm_omni_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_omni_perception::OmniPerceptionNode)
