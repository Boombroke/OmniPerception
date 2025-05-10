// Copyright 2022 Chen Jun
// Copyright 2025 Boombroke
// Licensed under the MIT License.

#ifndef RM_OMNI_PERCEPTION_OMNI_PERCEPTION_NODE_HPP
#define RM_OMNI_PERCEPTION_OMNI_PERCEPTION_NODE_HPP

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "rm_omni_perception/omni_perception.hpp"
#include "rm_omni_perception/number_classifier.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/omni.hpp"
namespace rm_omni_perception{
    class OmniPerceptionNode : public rclcpp::Node{
        public:
        OmniPerceptionNode(const rclcpp::NodeOptions & options);

        private:
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);
        void imageCallbackLeft(const sensor_msgs::msg::Image::SharedPtr img_msg);
        void imageCallbackMid(const sensor_msgs::msg::Image::SharedPtr img_msg);
        void imageCallbackRight(const sensor_msgs::msg::Image::SharedPtr img_msg);

        std::unique_ptr<Detector> initDetector();
        std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

        void createDebugPublishers();
        void destroyDebugPublishers();

        void publishMarkers();

        std::unique_ptr<Detector> detector_;

        // Detected armors publisher
        auto_aim_interfaces::msg::Armors armors_msg_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

        // Omni publisher
        rclcpp::Publisher<auto_aim_interfaces::msg::Omni>::SharedPtr omni_left_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Omni>::SharedPtr omni_mid_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Omni>::SharedPtr omni_right_pub_;

        // Image subscrpition
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_left_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_mid_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_right_;

        // Visualization marker publisher
        visualization_msgs::msg::Marker armor_marker_;
        visualization_msgs::msg::Marker text_marker_;
        visualization_msgs::msg::MarkerArray marker_array_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        // Debug information
        bool debug_;
        std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
        image_transport::Publisher binary_img_pub_;
        image_transport::Publisher number_img_pub_;
        image_transport::Publisher result_img_pub_;
    };
} // namespace rm_omni_perception_node

#endif // RM_OMNI_PERCEPTION_OMNI_PERCEPTION_NODE_HPP