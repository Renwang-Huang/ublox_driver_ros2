#include <iomanip>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stdlib.h>
#include <time.h>
// #include <sys/procmgr.h>
// #include <unistd.h>
// #include <arpa/inet.h>
#include <rclcpp/rclcpp.h>

#include "parameter_manager.hpp"
#include "serial_handler.hpp"
#include "socket_handler.hpp"
#include "file_dumper.hpp"
#include "file_loader.hpp"
#include "ublox_message_processor.hpp"

static std::atomic<bool> interrupted(false);

int ack_flag = 0;
std::mutex ack_m;
std::condition_variable ack_cv;

//设置中断程序
void ctrl_c_handler(int s)
{
    interrupted = true;
}

//获取系统时间
std::string time_str()
{
    std::stringstream ss;
    std::time_t time_ptr;
    time_ptr = time(NULL);
    tm *tm_local = localtime(&time_ptr);
    ss << std::setw(4) << std::setfill('0') << tm_local->tm_year + 1900
       << '_' << std::setw(2) << std::setfill('0') << tm_local->tm_mon + 1
       << '_' << std::setw(2) << std::setfill('0') << tm_local->tm_mday
       << '_' << std::setw(2) << std::setfill('0') << tm_local->tm_hour
       << '_' << std::setw(2) << std::setfill('0') << tm_local->tm_min
       << '_' << std::setw(2) << std::setfill('0') << tm_local->tm_sec;
    return ss.str();
}

//检查数据有效性
void config_ack_callback(const uint8_t *data, size_t len)
{
    int ack_result = UbloxMessageProcessor::check_ack(data, len);
    if (ack_result != 0)
    {
        std::lock_guard<std::mutex> ack_lk(ack_m);
        ack_flag = ack_result;
    }
    ack_cv.notify_one();
}

//确认串口数据接收正常
bool config_receiver(std::shared_ptr<SerialHandler> serial, std::vector<RcvConfigRecord> &rcv_configs)
{
    const uint32_t rcv_config_buff_capacity = 8192;
    std::unique_ptr<uint8_t[]> rcv_config_buff(new uint8_t[rcv_config_buff_capacity]);
    memset(rcv_config_buff.get(), 0, rcv_config_buff_capacity);
    uint32_t msg_len = 0;
    UbloxMessageProcessor::build_config_msg(rcv_configs, rcv_config_buff.get(), msg_len);
    std::unique_lock<std::mutex> ack_lk(ack_m);
    ack_flag = 0;

    //绑定为串口回调函数
    serial->addCallback(std::bind(&config_ack_callback, 
        std::placeholders::_1, std::placeholders::_2));
    serial->writeRaw(rcv_config_buff.get(), msg_len);
    serial->startRead();

    //阻塞程序运行直到串口接收到有效数据
    ack_cv.wait(ack_lk, []{return ack_flag != 0;});
    serial->stop_read();
    ack_lk.unlock();

    if (ack_flag == 1)
        return true;   
    return false;
}

int main(int argc, char **argv)
{
    ::google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    if (!interrupted.is_lock_free())  return 10;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ublox_driver");

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    // procmgr_ability( 0, PROCMGR_AID_CLOCKSET );

    /*
    ROS1中的参数获取实现：
    std::string config_filepath;
    nh.getParam("config_file", config_filepath);
    */

    node->declare_parameter<std::string>("config_file", "default/config/path");
    
    std::string config_filepath;
    if (node->get_parameter("config_file", config_filepath))
    {
        RCLCPP_INFO(node->get_logger(), "Config file: %s", config_filepath.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'config_file' not found!");
        return 1;
    }

    // config_filepath = argv[1];
    ParameterManager &pm(ParameterManager::getInstance());
    pm.read_parameter(config_filepath);

    std::shared_ptr<SerialHandler> serial;
    std::shared_ptr<SocketHandler> socket;
    std::shared_ptr<FileLoader> file_loader;
    std::shared_ptr<FileDumper> file_dumper;
    std::shared_ptr<UbloxMessageProcessor> ublox_msg_processor;
    std::shared_ptr<SerialHandler> output_serial;

    //依据输出形式初始化实例
    if (pm.to_ros)
        ublox_msg_processor.reset(new UbloxMessageProcessor(nh));
    if (pm.to_file)
    {
        const std::string t_str = time_str();
        const std::string dump_filepath = pm.dump_dir + "/" + t_str + ".ubx";
        file_dumper.reset(new FileDumper(dump_filepath));
    }
    if (pm.to_serial)
    {
        output_serial.reset(new SerialHandler(pm.output_serial_port, pm.serial_baud_rate));
    }
    
    //实时数据来源与离线数据来源对应的处理程序
    if (pm.online)
    {
        serial.reset(new SerialHandler(pm.input_serial_port, pm.serial_baud_rate));
        
        //该功能目前暂时不支持
        if (pm.config_receiver_at_start)
        {
            if (config_receiver(serial, pm.receiver_configs))
                LOG(INFO) << "Successfully configured the receiver.";
            else
                LOG(FATAL) << "Error occurs when configuring the receiver.";
        }
        
        if (pm.input_rtcm)
        {
            socket.reset(new SocketHandler("localhost", pm.rtcm_tcp_port));
            socket->addCallback(std::bind(&SerialHandler::writeRaw, serial.get(), 
                std::placeholders::_1, std::placeholders::_2, pm.IO_TIMEOUT_MS));
            socket->startRead();
        }
        
        if (pm.to_ros)
            serial->addCallback(std::bind(&UbloxMessageProcessor::process_data, 
                ublox_msg_processor.get(), std::placeholders::_1, std::placeholders::_2));
            
        if (pm.to_file)
            serial->addCallback(std::bind(&FileDumper::process_data, file_dumper.get(), 
                std::placeholders::_1, std::placeholders::_2));
        
        if (pm.to_serial)
            serial->addCallback(std::bind(&SerialHandler::writeRaw, output_serial.get(), 
                std::placeholders::_1, std::placeholders::_2, pm.IO_TIMEOUT_MS));

        serial->startRead();
    }
    else
    {
        file_loader.reset(new FileLoader(pm.ubx_filepath, pm.serial_baud_rate));

        if (pm.to_ros)
            file_loader->addCallback(std::bind(&UbloxMessageProcessor::process_data, 
                ublox_msg_processor.get(), std::placeholders::_1, std::placeholders::_2));
            
        if (pm.to_file)
            file_loader->addCallback(std::bind(&FileDumper::process_data, file_dumper.get(), 
                std::placeholders::_1, std::placeholders::_2));
            
        if (pm.to_serial)
            file_loader->addCallback(std::bind(&SerialHandler::writeRaw, output_serial.get(), 
                std::placeholders::_1, std::placeholders::_2, pm.IO_TIMEOUT_MS));

        file_loader->startRead();
    }

    rclcpp::Rate loop(50);
    while (rclcpp::ok() && !interrupted)
    {
        rclcpp::spin_some(node);
        loop.sleep();
    }

    if (serial)
        serial->close();
    if (file_loader)
        file_loader->close();
    
    return 0;
}