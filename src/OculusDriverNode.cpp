
#include <sonar_driver/OculusDriverNode.h>

/// @brief Constructor for OculusDriverNode
/// @param nodeName Name of the ROS2 node
OculusDriverNode::OculusDriverNode(const std::string& nodeName) : rclcpp::Node(nodeName){

    pub_imgUniformRaw = this->create_publisher<sensor_msgs::msg::Image>("image_uniform_raw", 10);

    this->declare_parameter<std::string>("image_transport", "compressed");
    pub_img = image_transport::create_publisher(this, "image");
    pub_imgUniform = image_transport::create_publisher(this, "image_uniform");
    pub_imgCartesian = image_transport::create_publisher(this, "image_cartesian");

    pub_depth = this->create_publisher<geometry_msgs::msg::PointStamped>("depth", 10);
    pub_temperature = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    pub_orientation = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("orientation", 10);
    pub_configuration = this->create_publisher<sonar_driver_interfaces::msg::SonarConfiguration>("configuration", 10);
    pub_bearings = this->create_publisher<sonar_driver_interfaces::msg::SonarBearings>("bearings", 10);
    
    sonar_ = std::make_unique<OculusSonar>(cvBridgeShared_);

    UniformBearingCorrectorConfig initConfig(0, 0, 0.0, 0.0, 0.0);
    bearingCorrector_ = std::make_shared<UniformBearingCorrector>(initConfig);

    updateCommonHeader();
    cvBridgeUniform_->encoding = "mono8";
    cvBridgeCartesian_->encoding = "mono8";

    sub_reconfiguration = this->create_subscription<sonar_driver_interfaces::msg::SonarConfigurationChange>(
        "reconfigure", 10, std::bind(&OculusDriverNode::cb_reconfiguration, this, std::placeholders::_1)
    );

}

/// @brief Callback for simple ping result from sonar 
/// @param image 
void OculusDriverNode::cb_simplePingResult(std::unique_ptr<SonarImage>& image){
    updateCommonHeader();

    double angularResolution = (sonar_->getBearingTable().back() - sonar_->getBearingTable().front()) / (100.0 * image->width);
    spdlog::debug("back: {}. front: {}. Diff: {}. Angular: {}. Width: {}", sonar_->getBearingTable().back(), sonar_->getBearingTable().front(), (sonar_->getBearingTable().back() - sonar_->getBearingTable().front()), angularResolution, image->width);
    UniformBearingCorrectorConfig currentConfig(image->height, image->width, 
                                                sonar_->getMinimumRange(), sonar_->getMaximumRange(),
                                                angularResolution);

    if(!bearingCorrector_->hasSameConfig(currentConfig)){
        spdlog::info("Changed config: back: {}. front: {}. Diff: {}. Angular: {}. Width: {}", sonar_->getBearingTable().back(), sonar_->getBearingTable().front(), (sonar_->getBearingTable().back() - sonar_->getBearingTable().front()), angularResolution, image->width);
        currentConfig.setBearings(sonar_->getBearingTable());
        bearingCorrector_ = std::make_shared<UniformBearingCorrector>(currentConfig);
    }


    bearingCorrector_->rectifyImage(cvBridgeShared_->image, cvBridgeUniform_->image, cvBridgeCartesian_->image);
    
    publishImage();
    publishUniformImage();
    publishCartesianImage();
    publishCurrentConfig();

}

/// @brief Update common header with current timestamp
void OculusDriverNode::updateCommonHeader(){
    commonHeader_.frame_id = "sonar_0";
    commonHeader_.stamp = this->get_clock()->now();
}

/// @brief Publish raw sonar image
void OculusDriverNode::publishImage(){
    auto msg_img = cvBridgeShared_->toImageMsg();
    msg_img->header = commonHeader_;
    this->pub_img.publish(*msg_img);
}

/// @brief Publish cartesian-corrected sonar image
void OculusDriverNode::publishCartesianImage(){
    auto msg_cartesian = cvBridgeCartesian_->toImageMsg();
    msg_cartesian->header = commonHeader_;
    this->pub_imgCartesian.publish(*msg_cartesian);
}

/// @brief Publish uniform-corrected sonar image
void OculusDriverNode::publishUniformImage(){
    auto msg_uniform = cvBridgeUniform_->toImageMsg();
    msg_uniform->header = commonHeader_;
    this->pub_imgUniform.publish(*msg_uniform);
    this->pub_imgUniformRaw->publish(*msg_uniform);
}

/// @brief Publish additional information from OculusSonarImage
/// @param image Source image containing sensor data
void OculusDriverNode::publishAdditionalInformation1(OculusSonarImage &image){
    this->publishPressure(image.pressure);
    this->publishTemperature(image.temperature);
}

/// @brief Publish additional information from OculusSonarImage2
/// @param image Source image containing sensor data
void OculusDriverNode::publishAdditionalInformation2(OculusSonarImage2 &image){
   
    this->publishPressure(image.pressure);
    
    this->publishTemperature(image.temperature);

    geometry_msgs::msg::Vector3Stamped msg;
    msg.header = commonHeader_;
    msg.vector.x = image.heading;
    msg.vector.y = image.pitch;
    msg.vector.z = image.roll;
    this->pub_orientation->publish(msg);
}

/// @brief Publish depth as PointStamped in ENU frame
/// @param pressure Pressure value to convert to depth
void OculusDriverNode::publishPressure(double pressure){
    geometry_msgs::msg::PointStamped msg;
    msg.header = commonHeader_;
    msg.point.x = 0.0;
    msg.point.y = 0.0;
    msg.point.z = -pressure / 10.0;  // Convert pressure to depth (negative)
    this->pub_depth->publish(msg);
}

/// @brief Publish temperature measurement
/// @param temperature Temperature value to publish
void OculusDriverNode::publishTemperature(double temperature){
    sensor_msgs::msg::Temperature msg;
    msg.header = commonHeader_;
    msg.temperature = temperature;
    this->pub_temperature->publish(msg);
}

/// @brief Publish current sonar configuration and bearings
void OculusDriverNode::publishCurrentConfig(){
    sonar_driver_interfaces::msg::SonarConfiguration configuration;

    configuration.header = commonHeader_;

    // Fill message with configuration data from sonar
    configuration.fire_mode = sonar_->getFireMode();
    configuration.frequency = sonar_->getOperatingFrequency();
    configuration.ping_rate = sonar_->getPingRate();
    configuration.beam_count = sonar_->getBeamCount();
    configuration.beam_separation = sonar_->getBeamSeparation();
    configuration.min_range = sonar_->getMinimumRange();
    configuration.max_range = sonar_->getMaximumRange();
    configuration.current_range = sonar_->getCurrentRange();
    configuration.range_resolution = sonar_->getRangeResolution();
    configuration.range_count = sonar_->getRangeBinCount();
    configuration.horz_fov = sonar_->getHorzFOV();
    configuration.vert_fov = sonar_->getVertFOV();
    configuration.angular_resolution = sonar_->getAngularResolution();
    configuration.gain = sonar_->getCurrentGain();
    configuration.gain_assist = sonar_->gainAssistEnabled();
    configuration.gamma = sonar_->getGamma();
    configuration.speed_of_sound = sonar_->getSpeedOfSound();
    configuration.salinity = sonar_->getSalinity();
    configuration.temperature = sonar_->getTemperature();
    configuration.pressure = sonar_->getPressure();
    configuration.net_speed_limit = sonar_->getNetworkSpeedLimit();

    // Publish the message on the OculusDriverNode
    pub_configuration->publish(configuration);

    // Publish the bearing table 
    sonar_driver_interfaces::msg::SonarBearings msg_bearings;
    msg_bearings.header = commonHeader_;
    msg_bearings.bearings = sonar_->getBearingTable();
    this->pub_bearings->publish(msg_bearings);
    
}

/// @brief Callback for sonar reconfiguration requests
/// @param msg Configuration change message
void OculusDriverNode::cb_reconfiguration(const sonar_driver_interfaces::msg::SonarConfigurationChange::SharedPtr msg){
    sonar_->configure(
        msg->fire_mode,
        msg->range,
        msg->gain,
        msg->speed_of_sound,
        msg->salinity,
        msg->gain_assist,
        msg->gamma,
        msg->net_speed_limit
    );

    sonar_->setPingRate(msg->ping_rate);
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    std::shared_ptr<OculusDriverNode> node = std::make_shared<OculusDriverNode>("OculusDriverNode");

    node->sonar_->findAndConnect();            // This starts the thread that will process new images
    if (node->sonar_->getState() != SonarState::Connected){
        exit(EXIT_FAILURE);
    }

    node->sonar_->configure(2, 10.0, 80.0, 0.0, 0.0, true, 255, 0xff);
    node->sonar_->setPingRate(0);
    
    SonarCallback callback = [&node](std::unique_ptr<SonarImage>& image) -> void {
        node->cb_simplePingResult(image);
    };

    node->sonar_->registerCallback(callback);

    while (rclcpp::ok()){
        rclcpp::spin_some(node);
        node->sonar_->fire();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    rclcpp::shutdown();

    node->sonar_->disconnect();

    exit(EXIT_SUCCESS);
}
