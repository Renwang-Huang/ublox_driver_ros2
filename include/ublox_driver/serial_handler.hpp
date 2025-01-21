#ifndef SERIAL_HANDLER_HPP_
#define SERIAL_HANDLER_HPP_

#include <iostream>
#include <vector>
#include <memory>
#include <functional>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "parameter_manager.hpp"

class SerialHandler
{
    public:
        SerialHandler(std::string port, unsigned int baud_rate, unsigned int buf_size=8192);
        SerialHandler(const SerialHandler&) = delete;
        SerialHandler& operator=(const SerialHandler&) = delete;
        ~SerialHandler();
        void addCallback(std::function<void (const uint8_t*, size_t)> callback);
        void startRead();
        void stop_read();
        void write(const std::string &str, uint32_t timeout_ms=50);
        void writeRaw(const uint8_t *data, size_t len, uint32_t timeout_ms=50);
        void close();

    private:
        void read_handler(const boost::system::error_code& error,
            std::size_t bytes_transferred);
        void write_callback(const boost::system::error_code& error, std::size_t bytes_transferred);
        void wait_callback(const boost::system::error_code& error);
        std::string port_;
        std::unique_ptr<uint8_t[]> data_buf_;
        boost::asio::io_service io_service_, timer_io_service_;
        boost::asio::serial_port serial_;
        boost::asio::io_service::work keep_working_task_;
        boost::asio::deadline_timer write_timeout_;
        std::vector<std::function<void (const uint8_t*, size_t)>> callbacks_;
        uint32_t MSG_HEADER_LEN;
};

#endif
