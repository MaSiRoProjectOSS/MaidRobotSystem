/**
 * @file FS.h
 * @brief
 * @version 0.23.12
 * @date 2024-01-13
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#ifndef FS_H
#define FS_H

#include "stab/api/String.h"

#include <Arduino.h>

namespace fs
{

#define FILE_READ   "r"
#define FILE_WRITE  "w"
#define FILE_APPEND "a"

enum SeekMode
{
    SeekSet = 0,
    SeekCur = 1,
    SeekEnd = 2
};

class File {
public:
    File(){};

    size_t write(uint8_t)
    {
        return 0;
    }
    size_t write(const uint8_t *buf, size_t size)
    {
        return 0;
    }
    int available()
    {
        return 0;
    }
    int read()
    {
        return 0;
    }
    int peek()
    {
        return 0;
    }
    void flush()
    {
    }
    size_t read(uint8_t *buf, size_t size)
    {
        return 0;
    }
    size_t readBytes(char *buffer, size_t length)
    {
        return read((uint8_t *)buffer, length);
    }

    bool seek(uint32_t pos, SeekMode mode)
    {
        return true;
    }
    bool seek(uint32_t pos)
    {
        return seek(pos, SeekSet);
    }
    size_t position()
    {
        return 0;
    }
    size_t size()
    {
        return 0;
    }
    bool setBufferSize(size_t size)
    {
        return 0;
    }
    void close()
    {
    }
    operator bool()
    {
        return true;
    }
    time_t getLastWrite()
    {
        return 0;
    }
    const char *path()
    {
        return "";
    }
    const char *name()
    {
        return "";
    }

    boolean isDirectory(void)
    {
        return false;
    }
    boolean seekDir(long position)
    {
        return false;
    }
    File openNextFile(const char *mode = FILE_READ)
    {
        return File();
    }
    String getNextFileName(void)
    {
        return "";
    }
    String getNextFileName(boolean *isDir)
    {
        return "";
    }
    void rewindDirectory(void)
    {
    }
    String readStringUntil(String value)
    {
        return "";
    }
    String readStringUntil(char value)
    {
        return "";
    }
    void printf(...)
    {
    }
};

class FS {
public:
    FS()
    {
    }

    File open(const char *path, const char *mode = FILE_READ, const bool create = false)
    {
        return File();
    }
    File open(const String &path, const char *mode = FILE_READ, const bool create = false)
    {
        return File();
    }

    bool exists(const char *path)
    {
        return true;
    }
    bool exists(const String &path)
    {
        return true;
    }

    bool remove(const char *path)
    {
        return true;
    }
    bool remove(const String &path)
    {
        return true;
    }

    bool rename(const char *pathFrom, const char *pathTo)
    {
        return true;
    }
    bool rename(const String &pathFrom, const String &pathTo)
    {
        return true;
    }

    bool mkdir(const char *path)
    {
        return true;
    }
    bool mkdir(const String &path)
    {
        return true;
    }

    bool rmdir(const char *path)
    {
        return true;
    }
    bool rmdir(const String &path)
    {
        return true;
    }

protected:
};

} // namespace fs

#endif //FS_H
