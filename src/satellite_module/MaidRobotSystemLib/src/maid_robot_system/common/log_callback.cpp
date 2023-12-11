/**
 * @file log_callback.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Log output for common use
 * @version 0.23.1
 * @date 2023-01-14
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "maid_robot_system/common/log_callback.hpp"

#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>

namespace maid_robot_system
{
namespace common
{

///////////////////////////////////////////////////////////////////
// public function
///////////////////////////////////////////////////////////////////
void LogCallback::happened_message(OUTPUT_LOG_LEVEL level, const char *message, const char *function_name, const char *file_name, int line)
{
    if (nullptr != this->_callback_message) {
        if (true == this->_output_log) {
            if (this->_level <= level) {
#if NDEBUG
                this->_callback_message(level, message, this->_log_information_null);
#else
                this->_callback_message(level, message, log_information(function_name, file_name, line));
#endif
            }
        }
    }
}

///////////////////////////////////////////////////////////////////
// short command
///////////////////////////////////////////////////////////////////
void LogCallback::log_trace(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE, message, function_name, file_name, line);
}
void LogCallback::log_debug(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG, message, function_name, file_name, line);
}
void LogCallback::log_info(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, message, function_name, file_name, line);
}
void LogCallback::log_message(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE, message, function_name, file_name, line);
}
void LogCallback::log_warn(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN, message, function_name, file_name, line);
}
void LogCallback::log_error(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ERROR, message, function_name, file_name, line);
}
void LogCallback::log_fatal(const char *message, const char *function_name, const char *file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, message, function_name, file_name, line);
}

///////////////////////////////////////////////////////////////////
// printf function
///////////////////////////////////////////////////////////////////
size_t LogCallback::log_printf(OUTPUT_LOG_LEVEL level, log_information info, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);
    if (len < 0) {
        va_end(arg);
        return 0;
    };
    if ((unsigned long long)len >= sizeof(loc_buf)) {
        temp = (char *)malloc(len + 1);
        if (temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len + 1, format, arg);
    }
    va_end(arg);
    // len = write((uint8_t *)temp, len);const char * message
    this->happened_message(level, temp, info.function_name, info.file_path, info.file_line);
    if (temp != loc_buf) {
        free(temp);
    }
    return len;
}

///////////////////////////////////////////////////////////////////
// setting
///////////////////////////////////////////////////////////////////
#if LOG_CALLBACK_DEFAULT_FUNCTION
void LogCallback::set_output_for_default_function()
{
    this->set_callback_message(&LogCallback::default_output_message);
}
#endif

void LogCallback::config_enable()
{
    this->config_log(true);
}
void LogCallback::config_disable()
{
    this->config_log(false);
}
void LogCallback::config_log(bool enable)
{
    this->_output_log = enable;
}
void LogCallback::config_level(OUTPUT_LOG_LEVEL level)
{
    this->_level = level;
}

///////////////////////////////////////////////////////////////////
// callback
///////////////////////////////////////////////////////////////////
bool LogCallback::hook_callback_message(MessageFunction callback)
{
    return true;
}

bool LogCallback::set_callback_message(MessageFunction callback)
{
    bool result = false;
    result      = hook_callback_message(callback);
    if (true == result) {
        this->_callback_message = callback;
    }
    return result;
}

#if LOG_CALLBACK_DEFAULT_FUNCTION
void LogCallback::default_output_message(LogCallback::OUTPUT_LOG_LEVEL level, //
                                         const char *message,
                                         LogCallback::log_information info)
{
    char buffer[1025]   = { 0 };
    unsigned long tm    = millis();
    unsigned long tm_s  = tm / 1000;
    unsigned long tm_ms = tm % 1000;

    if (level == LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE) {
        sprintf(buffer, "%s\n", message);
#if 0
    } else if (level >= LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE) {
        return;
#endif
    } else if (level >= LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN) {
        if (0 < info.file_line) {
            sprintf(buffer, "[%7lu.%03lu] [Error][%s() :  %s:%d] : %s\n", tm_s, tm_ms, info.function_name, info.file_path, info.file_line, message);
        } else {
            sprintf(buffer, "[%7lu.%03lu] [Error] : %s\n", tm_s, tm_ms, message);
        }
    } else {
        if (0 < info.file_line) {
            sprintf(buffer, "[%7lu.%03lu] [     ][%s() :  %s:%d] : %s\n", tm_s, tm_ms, info.function_name, info.file_path, info.file_line, message);
        } else {
            sprintf(buffer, "[%7lu.%03lu] [     ] : %s\n", tm_s, tm_ms, message);
        }
    }
    printf("%s", buffer);
}
#endif

///////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////
LogCallback::LogCallback()
{
}

LogCallback logger;

} // namespace common
} // namespace maid_robot_system
