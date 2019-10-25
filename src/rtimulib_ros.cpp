// RTIMULib ROS Node
// Copyright (c) 2017, Romain Reignier
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <RTIMULib.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <memory>

static const double G_TO_MPSS = 9.80665;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rtimulib_node");

    RCLCPP_INFO(node->get_logger(), "Imu driver is now running");

    std::string calibration_file_path;
    // if(!nh.getParam("calibration_file_path", calibration_file_path))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "The calibration_file_path parameter must be set to use a "
    //              "calibration file.");
    // }

    std::string calibration_file_name = "RTIMULib";
    // if(!nh.getParam("calibration_file_name", calibration_file_name))
    // {
    //    ROS_WARN_STREAM("No calibration_file_name provided - default: "
    //                    << calibration_file_name);
    // }

    std::string frame_id = "imu_link";
    // if(!nh.getParam("frame_id", frame_id))
    // {
    //    ROS_WARN_STREAM("No frame_id provided - default: " << frame_id);
    //}

    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);

    // Load the RTIMULib.ini config file
    RTIMUSettings *settings = new RTIMUSettings(calibration_file_path.c_str(),
                                                calibration_file_name.c_str());

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
        RCLCPP_ERROR(node->get_logger(), "No Imu found");
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    imu->setSlerpPower(0.02);
    // Enable the sensors
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    sensor_msgs::msg::Imu imu_msg;

    auto imu_pub_timer = node->create_wall_timer(
      std::chrono::duration<double>(imu->IMUGetPollInterval() / 1000.0),
      [&](){
        if (imu->IMURead())
        {
            RTIMU_DATA imu_data = imu->getIMUData();

            imu_msg.header.stamp = node->now();
            imu_msg.header.frame_id = frame_id;

            imu_msg.orientation.x = imu_data.fusionQPose.x(); 
            imu_msg.orientation.y = imu_data.fusionQPose.y(); 
            imu_msg.orientation.z = imu_data.fusionQPose.z(); 
            imu_msg.orientation.w = imu_data.fusionQPose.scalar(); 

            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();

            imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
            imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
            imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

            imu_pub->publish(imu_msg);
        }
      }
    );
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
