#pragma once

#include <stdint.h>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <sonar_driver_interfaces/msg/sonar_configuration.hpp>
#include <sonar_driver_interfaces/msg/sonar_configuration_change.hpp>
#include <sonar_driver_interfaces/msg/sonar_bearings.hpp>   


#include <sonar_driver/sonardevices/Sonar.h>
#include <sonar_driver/sonardevices/OculusSonar.h>

using std::placeholders::_1;

class SonarTimestampInserter : public rclcpp::Node
{
public:
    SonarTimestampInserter(const std::string& nodeName);
    
    std::unique_ptr<OculusSonar> sonar_;


    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
    rclcpp::Publisher<sonar_driver_interfaces::msg::SonarConfiguration>::SharedPtr pub_configuration_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Subscription<sonar_driver_interfaces::msg::SonarConfiguration>::SharedPtr sub_configuration_;

    void cb_simplePingResult(std::unique_ptr<SonarImage>& image);

    bool publishIt = false;

protected:

    void cb_img(sensor_msgs::msg::Image::SharedPtr msg);
    void cb_configuration(sonar_driver_interfaces::msg::SonarConfiguration::SharedPtr msg);


    std_msgs::msg::Header commonHeader_;

};


SonarTimestampInserter::SonarTimestampInserter(const std::string& nodeName) :
    Node(nodeName){
    pub_img_ = this->create_publisher<sensor_msgs::msg::Image>("image_out", 10);
    pub_configuration_ = this->create_publisher<sonar_driver_interfaces::msg::SonarConfiguration>("configuration_out", 10);
    
    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>("image_in", 10, std::bind(&SonarTimestampInserter::cb_img, this, _1));
    sub_configuration_ = this->create_subscription<sonar_driver_interfaces::msg::SonarConfiguration>("configuration_in", 10, std::bind(&SonarTimestampInserter::cb_configuration, this, _1) );

}


void SonarTimestampInserter::cb_img(sensor_msgs::msg::Image::SharedPtr msg) {
    commonHeader_.stamp = this->get_clock()->now();
    msg->header = commonHeader_;
    pub_img_->publish(*msg);
}

void SonarTimestampInserter::cb_configuration(sonar_driver_interfaces::msg::SonarConfiguration::SharedPtr msg){
    msg->header = commonHeader_;
    pub_configuration_->publish(*msg);
}



int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<SonarTimestampInserter> node = std::make_shared<SonarTimestampInserter>("SonarTimestampInserter");
    
    printf("SonarTimestampInserter: Starting Loop\n");

    while (rclcpp::ok()){
        rclcpp::spin_some(node);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    rclcpp::shutdown();

    node->sonar_->disconnect();

    exit(EXIT_SUCCESS);
}
