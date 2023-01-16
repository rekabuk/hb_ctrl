#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <pigpiod_if2.h>

//using std::placeholders::_1;

class HB_Ctrl : public rclcpp::Node
{
  public:
    HB_Ctrl() : Node("HB Control")
    {
      gpio_drver = pigpio_start(NULL, NULL);
      
      if (gpio_drver >= 0)
      {
        set_mode(gpio_drver, pin, PI_OUTPUT);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    "joy", rclcpp::QoS(10), std::bind(&HB_Ctrl::topic_callback, this, std::placeholders::_1));
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "cannot connect pigpiod");
        rclcpp::shutdown();
        exit(1);
      }

    }

  private:
    int gpio_drver;
    int pin = 17;

    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) const
    {
        if (joy_msg->buttons[5])
        {
          gpio_write(gpio_drver, pin, 1);
          RCLCPP_INFO(this->get_logger(), "Button 5 down, Write 1 on GPIO-%d", pin);
        }
        if (joy_msg->buttons[0])
        {
          gpio_write(gpio_drver, pin, 0);
          RCLCPP_INFO(this->get_logger(), "Button 5 ip, Write 0 on GPIO-%d", pin);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HB_Ctrl>());
  rclcpp::shutdown();
  return 0;
}