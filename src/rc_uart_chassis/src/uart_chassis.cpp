#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UartCommander : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    UartCommander(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
	FrameSubscribe_ = this->create_subscription<std_msgs::msg::String>("OptimalFrame", 10, std::bind(&UartCommander::FrameCallback, this, std::placeholders::_1));
    }

private:
    // 声明节点
    // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr FrameSubscribe_;

    void FrameCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<UartCommander>("UartCommander");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

