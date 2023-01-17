#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <pigpiod_if2.h>

//using std::placeholders::_1;

class HB_Ctrl : public rclcpp::Node
{
  public:
    HB_Ctrl() : Node("HB_Control")
    {
      gpio_drver = pigpio_start(NULL, NULL);
      
      if (gpio_drver >= 0)
      {
        set_mode(gpio_drver, gpio, PI_OUTPUT);
        gpio_write(gpio_drver, gpio, 1);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    "joy", rclcpp::QoS(10), std::bind(&HB_Ctrl::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Connected to pigpiod");
     }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Cannot connect pigpiod");
        rclcpp::shutdown();
        exit(1);
      }

    }

  private:
    int gpio_drver;
    int gpio = 17;    
    int hb_button = 5;

    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) const
    {
        static int on_count = 0;

        if (joy_msg->buttons[hb_button] == 1)
        {
          gpio_write(gpio_drver, gpio, 0);
          on_count = 2;
          RCLCPP_INFO(this->get_logger(), "Button %d down, Write 1 on GPIO-%d", hb_button, gpio);
        }
        // We only want to switch off the relay if it is on, otherwise we will try
        // to switch it off everytime the joy_node sends a message and that would be wastefull
        // Switch relay off twice just to be sure
        if (joy_msg->buttons[hb_button] == 0)
        {
          if (on_count > 0)
          {
            on_count--;
            gpio_write(gpio_drver, gpio, 1);
            RCLCPP_INFO(this->get_logger(), "Button %d up, Write 0 on GPIO-%d", hb_button, gpio);
          }
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