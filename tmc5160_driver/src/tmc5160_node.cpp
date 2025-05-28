#include "rclcpp/rclcpp.hpp"
#include "tmc5160_driver/tmc5160_driver.hpp"
#include <memory>

class TMC5160Node : public rclcpp::Node {
public:
  TMC5160Node() : Node("tmc5160_node") {
    RCLCPP_INFO(this->get_logger(), "TMC5160 Node started");
    
    // Get parameters
    std::string spi_device = this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
    int cs_gpio_pin = this->declare_parameter<int>("cs_gpio_pin", 29);  // Default to pin 29 for DRV1
    
    RCLCPP_INFO(this->get_logger(), "Using SPI device: %s with CS GPIO pin: %d", 
                spi_device.c_str(), cs_gpio_pin);
                
    // Initialize the driver with both parameters
    if (!driver_.initialize(spi_device, cs_gpio_pin)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize TMC5160 driver");
      return;
    }
    
    // Create a timer to periodically check motor status
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&TMC5160Node::timer_callback, this));
  }

  ~TMC5160Node() {
    driver_.shutdown();
  }

private:
  void timer_callback() {
    // Get current position
    int32_t position = driver_.getCurrentPosition();
    RCLCPP_INFO(this->get_logger(), "Current motor position: %d", position);
  }

  tmc5160_driver::TMC5160Driver driver_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TMC5160Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}