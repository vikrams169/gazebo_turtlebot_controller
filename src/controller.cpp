/**
 * @file controller.cpp
 * @author Vikram Setty (vikramsetty169@gmail.com)
 * @brief The node that controls the turtlebot's movement based on laser scan readings from the lidar sensor
 * @version 0.1
 * @date 2023-11-26
 * 
 * @copyright Copyright (c) 2023 Vikram Setty

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief The class to define the node where commands to the Turtlebot would be given based on the laser scan readings
 * 
 */
class ControllerNode : public rclcpp::Node{
 public:

  /**
   * @brief The constructor for the node class to initialize the publisher and subscriber
   * 
   */
  ControllerNode() : Node("controller") {
    this->declare_parameter("record_bag", 1);
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ControllerNode::subscriber_callback, this, _1));
  }

 private:

  /**
   * @brief The subscriber callback that takes the laser scan readings and decides whether to go forward or turn based on if an obstacle is in front of it
   * 
   * @param msg The laser scan readings by the turtlebot's lidar sensor
   * @return * void 
   */
  void subscriber_callback(const sensor_msgs::msg::LaserScan& msg){

    std::vector<float> range_values = static_cast<std::vector<float>>(msg.ranges) ;
    auto pub_vel = geometry_msgs::msg::Twist();
    int num_readings = range_values.size() ;
    int left = 7*num_readings/8 ;
    bool turn = false ;
    float stopping_threshold = 0.4 ;
    for(int i=0; i<(num_readings/4)+1; i++){
        int j = (left+i)%num_readings ;
        if(range_values[j] < stopping_threshold){
            turn = true ;
        }
    }
    if(turn){
        pub_vel.angular.z = 0.5 ;
    }
    else{
        pub_vel.linear.x = 0.3 ;
    }
    publisher_->publish(pub_vel) ;
    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      writer_->write(pub_vel, "/cmd_vel", time_stamp);
      writer_->write(msg, "scan", time_stamp);
    }

  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}