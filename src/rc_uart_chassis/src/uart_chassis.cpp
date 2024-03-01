#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>

class UartCommander : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    UartCommander(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
	    FrameSubscribe_ = this->create_subscription<std_msgs::msg::String>("OptimalFrame", 10, std::bind(&UartCommander::FrameCallback, this, std::placeholders::_1));
        init_OK = false;

        /* 初始化数据头和数据尾 */
        header[0] = 0x55;
        header[1] = 0xaa;
        ender[0] = 0x0d;
        ender[1] = 0x0a;

        /* 从参数服务器中获取参数 */
        this->declare_parameter<std::string>("dev", "");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<int>("time_out", 1000);
        this->declare_parameter<int>("hz", "100");

        this->get_parameter_or<std::string>("dev", dev, "");
        this->get_parameter_or<int>("baud", baud, "115200");
        this->get_parameter_or<int>("time_out", time_out, "1000");
        this->get_parameter_or<int>("hz", hz, "100");
    }

    void OpenUart ( void )
    {
        /* 开启串口模块 */
        try
        {
            ros_ser.setPort(dev);
            ros_ser.setBaudrate(baud);
            
            serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
            ros_ser.setTimeout(to);
            ros_ser.open();
            ros_ser.flushInput(); // 清空缓冲区数据
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
        }

        if(ros_ser.isOpen())
        {
            ros_ser.flushInput(); //清空缓冲区数据
            ROS_INFO_STREAM("Serial Port opened");
        }
        else
        {

        }

        
        while ( !init_OK )	
        {
            if ( ros_ser.available() )
            {
                std_msgs::String serial_data;
                string str_tem;
                //获取串口数据
                serial_data.data = ros_ser.read(ros_ser.available());
                str_tem = serial_data.data;
 
                if ( str_tem.find("OK", 0) ) // 在字符串 str_tem 中查找第一次出现的子串 “OK”。参数 0 表示从字符串的起始位置开始搜索
                    init_OK = true;
                else
                    ros_ser.flushInput(); //清空缓冲区数据
            }
            sleep(1);
        }
    }

    Bool AnalyUartReciveData( std_msgs::String& serialData )
    {
        uint8_t buf[500]; 
	    uint16_t dataLength = 0,i=0,len;
        unsigned char checkSum;
        unsigned char command;
	    uint8_t flag = 0;

        // if ( data_length < 1 || data_length > 500 )
        // {
        //     return false;
        // }

        for (i = 0; i < dataLength; i ++)
        {	
            buf[i] = serialData.data.at(i);
        }

        dataLength = buf[2];
        checkSum = getCrc8(buf, 3+dataLength);

        /* 检查信息校验值 */
        if ( checkSum != buf[3+dataLength] )                 //buf[10] 串口接收
        {
            // ROS_ERROR("Received data check sum error!");
            return false;
        }
        command = buf[3];
    }

private:
    // 声明节点
    // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr FrameSubscribe_;
    serial::Serial ros_ser;
    bool uart_recive_flag;
    std::string dev; // 串口号
    int baud, time_out, hz; // 波特率，延时时间，发布频率
    bool init_OK;

    const unsigned char header[2];
    const unsigned char ender[2];
    unsigned char ctrlflag = 0x07;


    void FrameCallback( const std_msgs::msg::String::SharedPtr msg )
    {
        SendCommand(msg.data);
	}	

    void SendCommand (std::string command)
    {
        unsigned char buf[8] = {0};
        int length = 2;

        /* 转换命令数据的类型 */
        const char * c_str = command.c_str();
        unsigned char result = static_cast<unsigned char>(c_str[0]);

        /* 设置消息头 */
        for (int i = 0; i < 2; i ++)
        {
            buf[i] = header[i];
        }

        /* 设置命令 */
        buf[2] = length;
        buf[3] = result;

        /* 预留控制命令 */
        buf[3+length-1] = ctrlflag;
        /* 设置校验位 */
        buf[3+length] = GetCrc8(buf, 3+length);
        buf[3+length+1] = ender[0];
        buf[3+length+2] = ender[1];

        /* 通过串口下发数据 */
        ros_ser.write(buf, 8);
    }

    unsigned char GetCrc8(unsigned char *ptr, unsigned short len)
    {
        unsigned char crc;
        unsigned char i;
        crc = 0;
        while(len--)
        {
            crc ^= *ptr++;
            for(i = 0; i < 8; i++)
            {
                if(crc&0x01)
                    crc=(crc>>1)^0x8C;
                else 
                    crc >>= 1;
            }
        }
        return crc;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<UartCommander>("UartCommander");
    node.OpenUart();
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

