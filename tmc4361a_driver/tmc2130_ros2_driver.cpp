//https://stackoverflow.com/questions/67894/why-do-we-need-extern-c-include-foo-h-in-c
extern "C" {
#include "TMC2130.h"
}
#include "rclcpp/rclcpp.hpp"

#define TMC2130_CHANNEL 0


TMC2130TypeDef tmc_dev;
TMC2130TypeDef * tmc_dev_ptr = &tmc_dev;

ConfigurationTypeDef tmc_config;
ConfigurationTypeDef * tmc_config_ptr = &tmc_config;

int32_t registerResetState;
int32_t * registerResetState_ptr = &registerResetState;


void initTMC(){

    tmc2130_init(tmc_dev_ptr, TMC2130_CHANNEL, tmc_config_ptr, &tmc2130_defaultRegisterResetState[0]);
    tmc2130_fillShadowRegisters(tmc_dev_ptr);
}


class MyNode: public rclcpp::Node
{
public:
    MyNode(): Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(),"Nodo prueba TMC2130");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }
private:
    
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(),"Hello %d",counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
}; 

int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MyNode>();

    initTMC();


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
