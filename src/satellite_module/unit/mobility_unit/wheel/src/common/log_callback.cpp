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
#include "common/log_callback.hpp"

///////////////////////////////////////////////////////////////////
// public function
///////////////////////////////////////////////////////////////////
void LogCallback::happened_message(OUTPUT_LOG_LEVEL level, const String message, const String function_name, const String file_name, int line)
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
void LogCallback::log_trace(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE, message, function_name, file_name, line);
}
void LogCallback::log_debug(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG, message, function_name, file_name, line);
}
void LogCallback::log_info(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, message, function_name, file_name, line);
}
void LogCallback::log_message(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE, message, function_name, file_name, line);
}
void LogCallback::log_warn(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN, message, function_name, file_name, line);
}
void LogCallback::log_error(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ERROR, message, function_name, file_name, line);
}
void LogCallback::log_fatal(const String message, const String function_name, const String file_name, int line)
{
    this->happened_message(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, message, function_name, file_name, line);
}

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
    if ((unsigned long long)len >= sizeof(loc_buf)) {
        temp = (char *)malloc(len + 1);
        if (temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len + 1, format, arg);
    }
    va_end(arg);
    // len = write((uint8_t *)temp, len);const String message
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
                                         const String message,
                                         LogCallback::log_information info)
{
    char buffer[300];
    unsigned long tm    = millis();
    unsigned long tm_s  = tm / 1000;
    unsigned long tm_ms = tm % 1000;

    if (level == LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE) {
        sprintf(buffer, "%s", message.c_str());
#if 0
    } else if (level >= LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE) {
        return;
#endif
    } else if (level >= LogCallback::OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN) {
        if (0 < info.file_line) {
            sprintf(buffer, "[%7ld.%03ld] [Error][%s() :  %s:%d] : %s", tm_s, tm_ms, info.function_name.c_str(), info.file_path.c_str(), info.file_line, message.c_str());
        } else {
            sprintf(buffer, "[%7ld.%03ld] [Error] : %s", tm_s, tm_ms, message.c_str());
        }
    } else {
        if (0 < info.file_line) {
            sprintf(buffer, "[%7ld.%03ld] [     ][%s() :  %s:%d] : %s", tm_s, tm_ms, info.function_name.c_str(), info.file_path.c_str(), info.file_line, message.c_str());
        } else {
            sprintf(buffer, "[%7ld.%03ld] [     ] : %s", tm_s, tm_ms, message.c_str());
        }
    }
    Serial.println(buffer);
}
#endif

///////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////
LogCallback::LogCallback()
{
}

LogCallback logger;
