/**
 * @file callback_if.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-18
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_INTERFACE_CONTROL_IF_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_INTERFACE_CONTROL_IF_HPP

namespace maid_robot_system
{
namespace common
{

template <typename ARGMENT>
class CallbackIF {
public:
    /**
     * @brief Declaration of callbacks
     */
    using CALLBACK_FUNC = void (*)(ARGMENT args);

    /**
     * @brief Set the callback function
     *
     * @param callback  Callback function to be called
     * @return true     Successfully
     * @return false    Failed
     */
    bool set_callback(CALLBACK_FUNC callback)
    {
        bool result = false;
        result      = hook_set_callback(callback);
        if (true == result) {
            this->_callback = callback;
        }
        return result;
    }

protected:
    /**
     * @brief
     *
     * @param args
     */
    void happened(ARGMENT args)
    {
        if (nullptr != this->_callback) {
            (void)this->_callback(args);
        }
    }

    /**
     * @brief Hooked when set_callback_message() is called
     * @attention Override if you want to hook it.
     *
     * @param callback argument of set_callback_message()
     * @return true  : The hook was successful.
     * @return false : NOT call set_callback_message().
     */
    virtual bool hook_set_callback(CALLBACK_FUNC callback)
    {
        return true;
    }

private:
    CALLBACK_FUNC _callback = nullptr; /*!< Callbacks to be called */
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
