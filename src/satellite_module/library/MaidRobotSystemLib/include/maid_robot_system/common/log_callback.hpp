/**
 * @file log_callback.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Log output for common use
 * @version 0.23.1
 * @date 2023-01-14
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_LOG_CALLBACK_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_LOG_CALLBACK_HPP

#include <stdio.h>

namespace maid_robot_system
{
namespace common
{

#ifndef __AVR__
#define LOG_CALLBACK_FUNCTION 0
#endif

#ifndef LOG_CALLBACK_DEFAULT_FUNCTION
/**
 * @brief Enable default function to output log in Serial
 */
#define LOG_CALLBACK_DEFAULT_FUNCTION 1
#endif

/**
 * @brief Log output for common use
 */
class LogCallback {
public:
    /**
     * @brief Define log level
     */
    typedef enum output_log_level_t
    {
        OUTPUT_LOG_LEVEL_ALL, /*!< Any logging levels that have been configured are logged at this log level. */
        OUTPUT_LOG_LEVEL_TRACE, /*!< The TRACE log level records all of the application's behaviour details. Its purpose is primarily diagnostic, and it is more granular and finer than the DEBUG log level. */
        OUTPUT_LOG_LEVEL_DEBUG, /*!< You are providing diagnostic information in a thorough manner with DEBUG. It's long and contains more information than you'll need when using the application. */
        OUTPUT_LOG_LEVEL_INFO,    /*!< INFO messages are similar to how applications normally behave. */
        OUTPUT_LOG_LEVEL_MESSAGE, /*!< When This log level signals operational messages. */
        OUTPUT_LOG_LEVEL_WARN, /*!< When an unexpected application issue has been identified, the WARN log level is used.  This indicates that you are unsure if the issue will recur or not. At this time, you may not notice any negative effects on your application. */
        OUTPUT_LOG_LEVEL_ERROR, /*!< This log level is used when a serious issue is preventing the application's functionalities from functioning properly. */
        OUTPUT_LOG_LEVEL_FATAL, /*!< The FATAL level of logging indicates that the application's situation is critical, such as when a critical function fails. */
        OUTPUT_LOG_LEVEL_OFF    /*!< Nothing is logged at this level of logging. */

    } OUTPUT_LOG_LEVEL;

    /**
     * @brief Information on log output destination
     */
    struct log_information {
    public:
        /**
         * @brief Construct a new log information object
         *
         * @param func : function name
         * @param path : file path
         * @param line : file line
         */
        log_information(const char *func = "", const char *path = "", int line = -1)
        {
            this->function_name = (char *)func;
            this->file_path     = (char *)path;
            this->file_line     = line;
        }
        char *function_name = (char *)""; /*!< function name */
        char *file_path     = (char *)""; /*!< file path */
        int file_line       = -1;         /*!< file line */
    };

public:
    /**
     * @brief Construct a new Log Callback object
     *
     */
    LogCallback();

#if LOG_CALLBACK_DEFAULT_FUNCTION
    /**
     * @brief Set the output for default function object
     *
     */
    void set_output_for_default_function();
#endif

    /**
     * @brief Enable output
     *
     */
    void config_enable();
    /**
     * @brief Disable output
     *
     */
    void config_disable();
    /**
     * @brief Arguments control output state
     *
     * @param enable : output state
     */
    void config_log(bool enable);
    /**
     * @brief Set the log level
     *
     * @param level Default : OUTPUT_LOG_LEVEL_INFO
     */
    void config_level(OUTPUT_LOG_LEVEL level = OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO);

public:
#if LOG_CALLBACK_FUNCTION
    /**
     * @brief Declaration of callbacks
     */
    typedef std::function<void(OUTPUT_LOG_LEVEL, const char *, log_information)> MessageFunction;
#else
    /**
     * @brief Declaration of callbacks
     */
    using MessageFunction = void (*)(OUTPUT_LOG_LEVEL, const char *, log_information);
#endif
    /**
     * @brief Set the callback message function
     *
     * @param callback  Callback function to be called
     * @return true     Successfully
     * @return false    Failed
     */
    bool set_callback_message(MessageFunction callback);

protected:
    /**
     * @brief Hooked when set_callback_message() is called
     * @attention Override if you want to hook it.
     *
     * @param callback argument of set_callback_message()
     * @return true  : The hook was successful.
     * @return false : NOT call set_callback_message().
     */
    virtual bool hook_callback_message(MessageFunction callback);

public:
#if LOG_CALLBACK_DEFAULT_FUNCTION
    /**
     * @brief Output functions provided by default
     *
     * @param level : log level
     * @param message : message text
     * @param info : log information
     */
    static void default_output_message(LogCallback::OUTPUT_LOG_LEVEL level, //
                                       const char *message,
                                       LogCallback::log_information info = LogCallback::log_information());
#endif
    /**
     * @brief Output message
     *
     * @param level : log level
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void happened_message(OUTPUT_LOG_LEVEL level, const char *message, const char *function_name = "", const char *file_name = "", int line = 0);

    /**
     * @brief Output at trace level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_trace(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);
    /**
     * @brief Output at debug level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_debug(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);
    /**
     * @brief Output at information level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_info(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);
    /**
     * @brief Output at message level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_message(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);
    /**
     * @brief Output at warning level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_warn(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);
    /**
     * @brief Output at error level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_error(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);
    /**
     * @brief Output at fatal level
     *
     * @param message : message text
     * @param function_name : function name
     * @param file_name : file name
     * @param line : file line
     */
    void log_fatal(const char *message, const char *function_name = "", const char *file_name = "", int line = -1);

    /**
     * @brief Output log with printf()
     * @warning Introduced on a trial basis
     *
     * @param level : log level
     * @param info : log information
     * @param format : message text
     * @param ... : additional Information
     * @return size_t : text size
     */
    size_t log_printf(OUTPUT_LOG_LEVEL level, log_information info, const char *format, ...);

private:
    MessageFunction _callback_message = nullptr; /*!< Callbacks to be called */

    bool _output_log        = false;                                   /*!< Output control flag   */
    OUTPUT_LOG_LEVEL _level = OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO; /*!< Output Level */
    log_information _log_information_null;                             /*!<  */
};

extern LogCallback logger; /*!< Instance Declaration */

/*!< MACRO : Output at trace level */
#define LOG_TRACE(mess) logger.log_trace((mess), __func__, __FILE__, __LINE__)
/*!< MACRO : Output at debug level */
#define LOG_DEBUG(mess) logger.log_debug((mess), __func__, __FILE__, __LINE__)
/*!< MACRO : Output at information level */
#define LOG_INFO(mess) logger.log_info((mess), __func__, __FILE__, __LINE__)
/*!< MACRO : Output at message level */
#define LOG_MESSAGE(mess) logger.log_message((mess), __func__, __FILE__, __LINE__)
/*!< MACRO : Output at warning level */
#define LOG_WARN(mess) logger.log_warn((mess), __func__, __FILE__, __LINE__)
/*!< MACRO : Output at error level */
#define LOG_ERROR(mess) logger.log_error((mess), __func__, __FILE__, __LINE__)
/*!< MACRO : Output at fatal level */
#define LOG_FATAL(mess) logger.log_fatal((mess), __func__, __FILE__, __LINE__)

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
