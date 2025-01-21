#ifndef FILE_LOADER_HPP_
#define FILE_LOADER_HPP_

#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>

#include "parameter_manager.hpp"

class FileLoader
{
    public:
        FileLoader(const std::string &filepath, uint32_t baud_rate=921600);
        FileLoader(const FileLoader&) = delete;
        FileLoader& operator=(const FileLoader&) = delete;
        ~FileLoader();
        void startRead();
        void close();
        void addCallback(std::function<void (const uint8_t*, size_t)> callback);

    private:
        void read_handler();

    private:
        std::string filepath_;
        uint32_t baud_rate_;
        std::ifstream ifs_;
        std::atomic<bool> paused_;
        std::atomic<bool> stopped_;
        std::atomic<bool> finished_;
        std::vector<std::function<void (const uint8_t*, size_t)>> callbacks_;
        std::thread read_thread_;
        uint32_t MSG_HEADER_LEN;
};

#endif
