/**
 * @file SPIFFS.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "SPIFFS.h"

namespace fs
{
SPIFFSFS::SPIFFSFS()
{
}
SPIFFSFS::~SPIFFSFS()
{
}
bool SPIFFSFS::begin(bool formatOnFail,
                     const char *basePath //
                     ,
                     uint8_t maxOpenFiles,
                     const char *partitionLabel)
{
    this->partitionLabel_ = (char *)partitionLabel;
    return true;
}
bool SPIFFSFS::format()
{
    return true;
}
size_t SPIFFSFS::totalBytes()
{
    return 0;
}
size_t SPIFFSFS::usedBytes()
{
    return 0;
}
void SPIFFSFS::end()
{
}

} // namespace fs

SPIFFSFS SPIFFS;
