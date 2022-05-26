#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
  {
    node.declare_parameter<T>(param_name); // for Galactic and newer
    if (!node.get_parameter(param_name, param_dest))
    {
      RCLCPP_ERROR(node.get_logger(), "Could not load param '%s'", param_name.c_str());
      return false;
    }
    else
    {
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
    }
    return true;
  }

class FakeLikarPublisher : public rclcpp::Node
{
public:
  FakeLikarPublisher()
  : Node("fake_lidar_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("topic", 10);

    RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
    bool loaded_successfully = true;
    loaded_successfully &= parse_param("rate", rate_, *this);
    loaded_successfully &= parse_param("blocking_distance", blocking_distance_, *this);
    loaded_successfully &= parse_param("frame_id", frame_id_, *this);

    // Check if all parameters were loaded correctly
    if (!loaded_successfully) {
      const std::string str = "Could not load all non-optional parameters. Shutting down.";
      RCLCPP_ERROR(get_logger(), "%s", str.c_str());
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_), std::bind(&FakeLikarPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::LaserScan();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = frame_id_;
    message.angle_min = -3.1241400241851807;
    message.angle_max = 3.1241400241851807;
    message.angle_increment = 0.008690236136317253;
    message.time_increment = 0.0;
    message.scan_time = 0.0;
    message.range_min = 0.15000000596046448;
    message.range_max = 14.0;

    const int number_of_elements = int((message.angle_max - message.angle_min) / message.angle_increment);

    for (int it = 0; it < number_of_elements ; it++) {
      message.ranges.push_back(blocking_distance_);
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing data");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::string frame_id_;
  double rate_;
  double blocking_distance_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLikarPublisher>());
  rclcpp::shutdown();
  return 0;
}
