#ifndef FILE_DUMPER_HPP_
#define FILE_DUMPER_HPP_

#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>

class FileDumper
{
    public:
        FileDumper(const std::string &filepath, uint32_t buf_len=8192);
        FileDumper(const FileDumper&) = delete;
        FileDumper& operator=(const FileDumper&) = delete;
        ~FileDumper();
        void process_data(const uint8_t *data, size_t len);

        static int createDirectoryIfNotExists(const char *path);

    private:
        std::string filepath_;
        uint32_t buf_len_;
        uint32_t buf_pos_;
        std::unique_ptr<uint8_t[]> data_buf_;
};

#endif
