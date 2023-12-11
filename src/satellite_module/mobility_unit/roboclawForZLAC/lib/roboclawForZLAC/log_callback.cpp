/**
 * @file log_callback.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-14
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "log_callback.hpp"

///////////////////////////////////////////////////////////////////
// public function
///////////////////////////////////////////////////////////////////
#pragma region command
void LogCallback::happened_message(OUTPUT_LOG_LEVEL level, const std::string message, const std::string function_name, const std::string file_name, int line)
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
#pragma endregion

///////////////////////////////////////////////////////////////////
// short command
///////////////////////////////////////////////////////////////////
#pragma region short_command
void LogCallback::log_trace(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE, message, function_name, file_name, line);
}
void LogCallback::log_debug(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG, message, function_name, file_name, line);
}
void LogCallback::log_info(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, message, function_name, file_name, line);
}
void LogCallback::log_message(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE, message, function_name, file_name, line);
}
void LogCallback::log_warn(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN, message, function_name, file_name, line);
}
void LogCallback::log_error(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ERROR, message, function_name, file_name, line);
}
void LogCallback::log_fatal(const std::string message, const std::string function_name, const std::string file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, message, function_name, file_name, line);
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// printf function
///////////////////////////////////////////////////////////////////
size_t LogCallback::printf(OUTPUT_LOG_LEVEL level, log_information info, const char *format, ...)
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
    if (len >= sizeof(loc_buf)) {
        temp = (char *)malloc(len + 1);
        if (temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len + 1, format, arg);
    }
    va_end(arg);
    //  len = write((uint8_t *)temp, len);const std::string message
    this->happened_message(level, temp, info.function_name, info.file_path, info.file_line);
    if (temp != loc_buf) {
        free(temp);
    }
    return len;
}

///////////////////////////////////////////////////////////////////
// setting
///////////////////////////////////////////////////////////////////
#pragma region command
void LogCallback::output_log_enable()
{
    this->output_log(true);
}
void LogCallback::output_log_disable()
{
    this->output_log(false);
}
void LogCallback::output_log(bool enable)
{
    this->_output_log = enable;
}
void LogCallback::output_log_level(OUTPUT_LOG_LEVEL level)
{
    this->_level = level;
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// callback
///////////////////////////////////////////////////////////////////
#pragma region callback
bool LogCallback::callback_message(MessageFunction callback)
{
    return true;
}

bool LogCallback::set_callback_message(MessageFunction callback)
{
    bool result = false;
    try {
        this->_callback_message = callback;
        result                  = callback_message(callback);
    } catch (...) {
    }
    return result;
}
#pragma endregion

///////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////
#pragma region constructor
LogCallback::LogCallback()
{
}

LogCallback::~LogCallback()
{
}
#pragma endregion