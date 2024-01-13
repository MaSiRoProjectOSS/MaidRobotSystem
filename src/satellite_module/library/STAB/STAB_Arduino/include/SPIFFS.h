/**
 * @file SPIFFS.h
 * @brief
 * @version 0.23.12
 * @date 2024-01-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef SPIFFS_H_
#define SPIFFS_H_

#include "FS.h"
#include "stab/arduino_types.hpp"
namespace fs
{

class SPIFFSFS : public FS {
public:
    SPIFFSFS();
    ~SPIFFSFS();
    bool begin(bool formatOnFail = false, const char *basePath = "/spiffs", uint8_t maxOpenFiles = 10, const char *partitionLabel = NULL);
    bool format();
    size_t totalBytes();
    size_t usedBytes();
    void end();

private:
    char *partitionLabel_;
};

} // namespace fs

using namespace fs;
extern SPIFFSFS SPIFFS;
#endif
