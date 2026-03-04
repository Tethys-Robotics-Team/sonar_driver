# ROS 2 Sonar Driver

## Description
ROS 2 driver for Blueprint Subsea Oculus imaging sonars. Provides real-time sonar data acquisition, image processing with uniform bearing correction, and dynamic configuration through ROS 2 topics and services.

## Supported Devices
- **Oculus M1200d** - Dual-frequency (750 kHz / 1.2 MHz) multi-beam imaging sonar
- **Oculus M3000d** - Dual-frequency (1.2 MHz / 2.1 MHz) high-resolution multi-beam imaging sonar

## Dependencies
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- image_transport & compressed_image_transport
- cv_bridge & libopencv-dev
- **[sonar_driver_interfaces](https://github.com/Tethys-Robotics-Team/sonar_driver_interfaces)** - Custom message definitions (Tethys Robotics AG)

