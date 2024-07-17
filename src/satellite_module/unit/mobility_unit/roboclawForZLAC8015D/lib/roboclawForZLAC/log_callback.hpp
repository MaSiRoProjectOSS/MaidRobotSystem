/**
 * @file log_callback.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-14
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef LOG_CALLBACK_HPP
#define LOG_CALLBACK_HPP
#include <Arduino.h>

class LogCallback {
public:
    typedef enum output_log_level
    {
        /**
          * @brief  Any logging levels that have been configured are logged at this log level.
          */
        OUTPUT_LOG_LEVEL_ALL,
        /**
          * @brief The TRACE log level records all of the application's behaviour details. Its purpose is primarily diagnostic, and it is more granular and finer than the DEBUG log level.
          */
        OUTPUT_LOG_LEVEL_TRACE,
        /**
          * @brief You are providing diagnostic information in a thorough manner with DEBUG. It's long and contains more information than you'll need when using the application.
          */
        OUTPUT_LOG_LEVEL_DEBUG,
        /**
          * @brief INFO messages are similar to how applications normally behave.
          */
        OUTPUT_LOG_LEVEL_INFO,
        /**
          * @briefWhen This log level signals operational messages
          */
        OUTPUT_LOG_LEVEL_MESSAGE,
        /**
          * @briefWhen an unexpected application issue has been identified, the WARN log level is used.  This indicates that you are unsure if the issue will recur or not. At this time, you may not notice any negative effects on your application.
          */
        OUTPUT_LOG_LEVEL_WARN,
        /**
          * @brief This log level is used when a serious issue is preventing the application's functionalities from functioning properly.
          */
        OUTPUT_LOG_LEVEL_ERROR,
        /**
          * @brief The FATAL level of logging indicates that the application's situation is critical, such as when a critical function fails.
          */
        OUTPUT_LOG_LEVEL_FATAL,
        /**
          * @brief Nothing is logged at this level of logging.
          */
        OUTPUT_LOG_LEVEL_OFF

    } OUTPUT_LOG_LEVEL;

    struct log_information {
    public:
        log_information(const std::string func = "", const std::string path = "", int line = -1)
        {
            this->function_name = func;
            this->file_path     = path;
            this->file_line     = line;
        }
        std::string function_name = "";
        std::string file_path     = "";
        int file_line             = -1;
    };

public:
    LogCallback();
    ~LogCallback();

    void output_log_enable();
    void output_log_disable();
    void output_log(bool enable);
    void output_log_level(OUTPUT_LOG_LEVEL level = OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO);

public:
    typedef std::function<void(OUTPUT_LOG_LEVEL, const std::string, log_information)> MessageFunction;

    /**
     * @brief Set the callback message function
     *
     * @param callback  Callback function to be called
     * @return true     Successfully
     * @return false    Failed
     */
    bool set_callback_message(MessageFunction callback);

protected:
    virtual bool callback_message(MessageFunction callback);

protected:
    void happened_message(OUTPUT_LOG_LEVEL level, const std::string message, const std::string function_name = "", const std::string file_name = "", int line = 0);

    void log_trace(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);
    void log_debug(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);
    void log_info(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);
    void log_message(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);
    void log_warn(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);
    void log_error(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);
    void log_fatal(const std::string message, const std::string function_name = "", const std::string file_name = "", int line = -1);

    size_t printf(OUTPUT_LOG_LEVEL level, log_information info, const char *format, ...);

private:
    MessageFunction _callback_message = nullptr;
    bool _output_log                  = false;
    OUTPUT_LOG_LEVEL _level           = OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO;
    log_information _log_information_null;
};

#endif
