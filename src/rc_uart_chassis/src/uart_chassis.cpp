#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>

#include <iostream>
#include <string>

class UartCommander : public rclcpp::Node
{
public:
    /* 构造函数,有一个参数为节点名称 */
    UartCommander(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
	    FrameSubscribe_ = this->create_subscription<std_msgs::msg::String>("optimal_frame", 10, std::bind(&UartCommander::FrameCallback, this, std::placeholders::_1));
        DropBallSubscribe_ = this->create_subscription<std_msgs::msg::Bool>("drop_ball", 10, std::bind(&UartCommander::DropBallCallback, this, std::placeholders::_1));
        PickBallSubscribe_ = this->create_subscription<std_msgs::msg::Bool>("pick_ball", 10, std::bind(&UartCommander::PickBallCallback, this, std::placeholders::_1));
        // timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UartCommander::timer_callback, this));
        init_OK = false;

        /* 初始化数据头和数据尾 */
        header[0] = 0x55;
        header[1] = 0xaa;
        ender[0] = 0x0d;
        ender[1] = 0x0a;

        /* 从参数服务器中获取参数 */
        this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<int>("time_out", 1000);
        this->declare_parameter<int>("hz", 100);

        this->get_parameter_or<std::string>("dev", dev, "/dev/ttyUSB0");
        this->get_parameter_or<int>("baud", baud, 115200);
        this->get_parameter_or<int>("time_out", time_out, 1000);
        this->get_parameter_or<int>("hz", hz, 100);

        rate_ = std::make_shared<rclcpp::Rate>(hz);
    }

    /**
     * @brief 完成串口初始化，并打开串口
     * @param 无
     * @retval 无
     */
    void OpenUart ( void )
    {
        /* 开启串口模块 */
        int x1 = 0;
        try
        {
            ros_ser.setPort(dev);
            ros_ser.setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
            ros_ser.setTimeout(to);
            ros_ser.open();
            ros_ser.flushInput(); // 清空缓冲区数据
            this->str = "初始化完毕.";
            std::cout << this->str << std::endl;
        }
        catch (serial::IOException& e)
        {
            this->str = "Unable to open port.";
            std::cout << this->str << std::endl;
        }

        /* 检查串口是否开启 */
        if(ros_ser.isOpen())
        {
            ros_ser.flushInput(); //清空缓冲区数据
            this->str = "Serial Port opened.";
            std::cout << this->str << std::endl;
        }
        else
        {
            this->str = "未成功打开串口.";
            std::cout << this->str << std::endl;
        }

        
        // while ( !init_OK )	
        // {
        //     if ( ros_ser.available() )
        //     {
        //         std_msgs::String serial_data;
        //         string str_tem;
        //         //获取串口数据
        //         serial_data.data = ros_ser.read(ros_ser.available());
        //         str_tem = serial_data.data;
 
        //         if ( str_tem.find("OK", 0) ) // 在字符串 str_tem 中查找第一次出现的子串 “OK”。参数 0 表示从字符串的起始位置开始搜索
        //             init_OK = true;
        //         else
        //             ros_ser.flushInput(); //清空缓冲区数据
        //     }
        //     sleep(1);
        // }
    }

    /**
     * @brief 辨析串口接收到的数据
     * @param serialData --- 串口数据
     * @retval --- 
     */
    bool AnalyUartReciveData( std_msgs::msg::String& serialData )
    {
        unsigned char buf[500]; 
	    uint16_t dataLength = 0, i = 0, len;
        unsigned char checkSum;
        unsigned char command;
	    uint8_t flag = 0;

        // if ( data_length < 1 || data_length > 500 )
        // {
        //     return false;
        // }
        dataLength = serialData.data.size();
        for (i = 0; i < dataLength; i ++)
        {	
            buf[i] = serialData.data.at(i);
        }

        /* 判断数据头是否正确 */
        if (buf[0] != header[0] && buf[1] != header[1])
            return false;

        if (buf[6] != ender[0] && buf[7] != ender[1])
            return false;

        dataLength = buf[2];
        checkSum = GetCrc8(buf, 3+dataLength);

        /* 检查信息校验值 */
        // if ( checkSum != buf[3+dataLength] )                 //buf[10] 串口接收
        // {
        //     // ROS_ERROR("Received data check sum error!");
        //     std::string str = "处理数据发生错误";
        //     std::cout << str << std::endl;
        //     return false;
        // }
        command = buf[3];
        str = "正在打印数据：";
        std::cout << static_cast<int>(buf[0]) << std::endl;
        std::cout << static_cast<int>(buf[1]) << std::endl;
        std::cout << static_cast<int>(buf[2]) << std::endl;
        std::cout << static_cast<int>(buf[3]) << std::endl;
        std::cout << static_cast<int>(buf[4]) << std::endl;
        std::cout << static_cast<int>(buf[5]) << std::endl;
        std::cout << static_cast<int>(buf[6]) << std::endl;
        std::cout << static_cast<int>(buf[7]) << std::endl;
        // std::string str = command;
        //std::string myString(1, static_cast<char>(command)); 
        //std::cout << myString << std::endl;
	    return true;
    }

    /* 公共变量 */
    serial::Serial ros_ser;
    bool uart_recive_flag;
    std::string str = "";
    std::shared_ptr<rclcpp::Rate> rate_;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr FrameSubscribe_; // 创建订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr DropBallSubscribe_; // 创建订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr PickBallSubscribe_;

    // rclcpp::TimerBase::SharedPtr timer;
    std::string dev; // 串口号
    int baud, time_out, hz; // 波特率，延时时间，发布频率
    bool init_OK;
    unsigned char header[2];
    unsigned char ender[2];
    unsigned char ctrlflag = 0x07;

    void PickBallCallback( const std_msgs::msg::Bool::SharedPtr msg )
    {
        
    }

    void DropBallCallback( const std_msgs::msg::Bool::SharedPtr msg )
    {
        
    }

    void FrameCallback( const std_msgs::msg::String::SharedPtr msg )
    {
        SendCommand(msg->data);
	}	

    void SendCommand ( std::string command )
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
        buf[3+length-1] = ctrlflag; // 4
        /* 设置校验位 */
        buf[3+length] = GetCrc8(buf, 3+length); // 5
        buf[3+length+1] = ender[0]; // 6
        buf[3+length+2] = ender[1]; //

        /* 通过串口下发数据 */
        ros_ser.write(buf, 8);
    }

    /**
     * @brief 计算CRC-8校验码。CRC（循环冗余校验）是一种常用的数据校验方法，用于检测数据在传输或存储过程中是否发生了错误
     * @param ptr --- 数组首地址
     * @param len --- 数组目前有效数据长度
     * @retval crc --- CRC-8校验码
     */
    unsigned char GetCrc8( unsigned char *ptr, unsigned short len )
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
    node->OpenUart();

    node->str = "开始接收串口数据";
    std::cout << node->str << std::endl;

    while (rclcpp::ok())
    {  
		if ( node->ros_ser.available() )
		{
			std_msgs::msg::String serial_data;

            node->str = "开始读数据";
            std::cout << node->str << std::endl;
			serial_data.data = node->ros_ser.read(node->ros_ser.available());
 
			node->uart_recive_flag = node->AnalyUartReciveData(serial_data);
			
			if ( node->uart_recive_flag )
			{
				node->uart_recive_flag = 0;
			}
			else
			{
				node->str = "本次没有接受到正确的数据";
                std::cout << node->str << std::endl;
			}
		}	 
		//rclcpp::spin(node);	
        node->rate_->sleep();
    }
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

