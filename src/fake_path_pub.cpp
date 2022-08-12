#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "fognav_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/home_position.hpp"

using namespace std::chrono_literals;

template <class T>
bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
{
  node.declare_parameter<T>(param_name); 
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

class FakePathPublisher : public rclcpp::Node
{
public:
  FakePathPublisher();

private:
  bool loaded_successfully = true;
  bool got_home_position_ = false;

  int publisher_type_;
  double angle_;
  double rate_;
  double step_;
  double speed_;
  double radius_;
  double height_;
  double duration_;
  std::string drone_name_;
  std::string geographic_global_frame_;

  double home_lat_;
  double home_lon_;
  double home_alt_;
  std::mutex mutex_home_position_;

  geometry_msgs::msg::PoseStamped uav_pose_;

  rclcpp::Time last_stamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<fognav_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_pos_subscription_;

  void home_position_callback(const px4_msgs::msg::HomePosition::UniquePtr msg);
  void timer_callback(void);

};

FakePathPublisher::FakePathPublisher() : Node("fake_path_publisher")
{
  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");

  loaded_successfully &= parse_param("rate", rate_, *this);
  loaded_successfully &= parse_param("speed", speed_, *this);
  loaded_successfully &= parse_param("height", height_, *this);
  loaded_successfully &= parse_param("duration", duration_, *this);
  loaded_successfully &= parse_param("pub_type", publisher_type_, *this);
  loaded_successfully &= parse_param("drone_name", drone_name_, *this);
  loaded_successfully &= parse_param("geographic_global_frame", geographic_global_frame_, *this);

  // Check if all parameters were loaded correctly
  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(get_logger(), "%s", str.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(), "-------------- Parameters loaded --------------");

  angle_ = 0.0;
  radius_ = 4.0;
  step_ = 30/180.0 * M_PI;

  home_lat_ = 0;
  home_lon_ = 0; 
  home_alt_ = 0;
  last_stamp_ = this->get_clock()->now();

  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_out", 10);
  trajectory_publisher_ = this->create_publisher<fognav_msgs::msg::Trajectory>("trajectory_out", 10);

  
  rclcpp::CallbackGroup::SharedPtr px4_home_position_subscriber_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  auto px4_home_position_subscriber_opt = rclcpp::SubscriptionOptions();
  px4_home_position_subscriber_opt.callback_group = px4_home_position_subscriber_cbg_;

  home_pos_subscription_ = this->create_subscription<px4_msgs::msg::HomePosition>("home_pos_in", rclcpp::SensorDataQoS(), std::bind(&FakePathPublisher::home_position_callback, this, std::placeholders::_1), px4_home_position_subscriber_opt);

  timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_), std::bind(&FakePathPublisher::timer_callback, this));
}

void FakePathPublisher::timer_callback()
{
  if(!got_home_position_)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 2000,  "Waiting for home position");
    return;
  }

  rclcpp::Time now = this->get_clock()->now();
  double dt = (now - last_stamp_).seconds();

  if(duration_ > 0)
  {
    nav_msgs::msg::Path path;

    path.header.stamp = now; 
    path.header.frame_id = geographic_global_frame_;

    for (int i = 1; i <= 10; i++) 
    {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = geographic_global_frame_;
      
      double current_angle = angle_;
      while(current_angle > 2*M_PI)
      {
        current_angle = current_angle - 2*M_PI;
      }

      pose_msg.pose.position.x = (radius_+0.1*angle_) * std::cos(current_angle+i*dt*step_);
      pose_msg.pose.position.y = (radius_+0.1*angle_) * std::sin(current_angle+i*dt*step_);
      pose_msg.pose.position.z = height_;
      path.poses.push_back(pose_msg);
    }
    duration_ = duration_ - dt;

    if (publisher_type_ == 0)
    {
      path_publisher_->publish(path);
      RCLCPP_INFO_ONCE(this->get_logger(), "Publishing path !!!");
    }
    else
    {
      fognav_msgs::msg::Trajectory trajectory_msg;
      trajectory_msg.header.stamp = now;
      trajectory_msg.droneid = drone_name_;
      { 
        std::scoped_lock lock(mutex_home_position_);
        
        trajectory_msg.datum.latitude = home_lat_;
        trajectory_msg.datum.longitude = home_lon_;
        trajectory_msg.datum.altitude = home_alt_;
      }
      int i = 0;
      for(auto& pose : path.poses)
      {
        trajectory_msg.path[i] = pose.pose.position;
        i++;
      }

      trajectory_publisher_->publish(trajectory_msg);
      RCLCPP_INFO_ONCE(this->get_logger(), "Publishing trajectory !!!");
    }
    angle_ = angle_ + dt*step_;

  }
  else
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Done");
  }
  last_stamp_ = now;
}

void FakePathPublisher::home_position_callback(const px4_msgs::msg::HomePosition::UniquePtr msg)
{
  std::scoped_lock lock(mutex_home_position_);
  home_lat_ = msg->lat;
  home_lon_ = msg->lon;
  home_alt_ = msg->alt;
  got_home_position_ = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakePathPublisher>());
  rclcpp::shutdown();
  return 0;
}
