#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class SphereControllerNode : public rclcpp::Node {
public:
  SphereControllerNode() : Node("sphere_controller_node"), start_time_(this->now()) {
    // Kinematic parameters
    this->declare_parameter("amplitude", 2.0);  // Meters
    this->declare_parameter("frequency", 1.0);  // Rad/s

    amplitude_ = this->get_parameter("amplitude").as_double();
    omega_ = this->get_parameter("frequency").as_double();

    // Publisher targeting Gazebo cmd_vel topic
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/oscillating_sphere/cmd_vel", 10);

    // Timer loop @ 50Hz (20ms)
    timer_ = this->create_wall_timer(20ms, std::bind(&SphereControllerNode::update_kinematics, this));

    RCLCPP_INFO(this->get_logger(), "Oscillating Sphere Controller Initialized.");
    RCLCPP_INFO(this->get_logger(), "Amplitude: %.2f m | Angular Freq: %.2f rad/s", amplitude_, omega_);
  }

private:
  void update_kinematics() {
    double t = (this->now() - start_time_).seconds();

    // Kinematic equations
    double pos_x = amplitude_ * std::sin(omega_ * t);
    double vel_x = amplitude_ * omega_ * std::cos(omega_ * t);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = vel_x;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;

    cmd_vel_pub_->publish(twist_msg);

    // Output status on endpoints and midpoints
    if (std::abs(vel_x) < 0.05) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "ENDPOINT STOP: Pos X = %.2f m | Vel X = %.2f m/s", pos_x, vel_x);
    } else if (std::abs(pos_x) < 0.1) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "MIDPOINT MAX SPEED: Pos X = %.2f m | Vel X = %.2f m/s", pos_x, vel_x);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  double amplitude_;
  double omega_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SphereControllerNode>());
  rclcpp::shutdown();
  return 0;
}