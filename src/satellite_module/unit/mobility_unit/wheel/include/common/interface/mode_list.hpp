/**
 * @file mode_list.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Define the mode of the system
 * @version 0.1
 * @date 2023-02-14
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef COMMON_INTERFACE_MODE_LIST_HPP
#define COMMON_INTERFACE_MODE_LIST_HPP

#include <stdio.h>

/**
 * @brief Define the mode of the system
 */
enum ModeList
{
    MODE_NOT_INITIALIZED,         /*!< No startup process has been done. */
    MODE_RUNNING,                 /*!< It is working. */
    MODE_FINISHED,                /*!< Stop processing has been executed. */
    MODE_ERROR_GENERAL,           /*!< [ERROR] General Error */
    MODE_ERROR_NOT_COMMUNICATION, /*!< [ERROR] Communication has not been established. */
    MODE_ERROR_CAN_NOT_STOP,      /*!< [ERROR] Stop processing could not be executed. */
};

/**
 * @brief ModeList to Text function
 *
 * @param mode : ModeList
 * @return char* : Text
 */
inline char *text_contorl_mode(ModeList mode)
{
    static char buffer[255] = { 0 };
    switch (mode) {
        case ModeList::MODE_NOT_INITIALIZED:
            sprintf(buffer, "Not initialized");
            break;
        case ModeList::MODE_RUNNING:
            sprintf(buffer, "Running");
            break;
        case ModeList::MODE_FINISHED:
            sprintf(buffer, "Finished");
            break;
        case ModeList::MODE_ERROR_GENERAL:
            sprintf(buffer, "<ERROR>Geneal");
            break;
        case ModeList::MODE_ERROR_NOT_COMMUNICATION:
            sprintf(buffer, "<ERROR>Not communication");
            break;
        case ModeList::MODE_ERROR_CAN_NOT_STOP:
            sprintf(buffer, "<ERROR>CAN'T SYSTEM STOP");
            break;

        default:
            sprintf(buffer, "UNKNOW");
            break;
    }
    return buffer;
}

#endif
