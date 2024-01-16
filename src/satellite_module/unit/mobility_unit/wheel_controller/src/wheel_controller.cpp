/**
 * @file wheel_controller.cpp
 * @brief
 * @version 0.23.12
 * @date 2024-01-06
 *
 * @copyright Copyright (c) MaSiRo Project. 2024-.
 *
 */
#include "wheel_controller.hpp"

#define ENABLE_PWM_SERVO 0

// =============================
// Constructor
// =============================
WheelController::WheelController(HardwareSerial *serial, int pwm_size, int pwm_address) : ModbusLibArduino(serial)
{
    this->pwm_servo         = new int[pwm_size];
    this->pwm_servo_request = new int[pwm_size];
    this->pwm_servo_count   = pwm_size;
    this->_load_setting_setting();
    this->_pwm = new Adafruit_PWMServoDriver(pwm_address);
}

WheelController::~WheelController(void)
{
    delete[] this->pwm_servo;
    delete[] this->pwm_servo_request;
}

// =============================
// Protected : Virtual Function
// =============================
bool WheelController::_init()
{
    bool result = this->_load_setting_setting();
    try {
#if ENABLE_PWM_SERVO
        result = this->_pwm->begin();
        this->_pwm->setOscillatorFrequency(this->_oscillator_frequency);
        this->_pwm->setPWMFreq(this->_pwm_freq);
#endif
    } catch (...) {
        result = false;
    }
    return result;
}
MessageFrame WheelController::_reception(MessageFrame frame)
{
    this->count_reception++;
    /*
| Function type |               |                                                 | Function name                    | Function code | Comment     |
| ------------- | ------------- | ----------------------------------------------- | -------------------------------- | :-----------: | ----------- |
| Data Access   | Bit access    | Physical Discrete Inputs                        | Read Discrete Inputs             |    2(0x02)    | --          |
| ^             | ^             | Internal Bits or Physical Coils                 | Read Coils                       |    1(0x01)    | --          |
| ^             | ^             | ^                                               | Write Single Coil                |    5(0x05)    | --          |
| ^             | ^             | ^                                               | Write Multiple Coils             |   15(0x0F)    | --          |
| ^             | 16-bit access | Physical Input Registers                        | Read Input Registers             |    4(0x04)    | --          |
| ^             | ^             | Internal Registers or Physical Output Registers | Read Multiple Holding Registers  |    3(0x03)    | --          |
| ^             | ^             | ^                                               | Write Single Holding Register    |    6(0x06)    | --          |
| ^             | ^             | ^                                               | Write Multiple Holding Registers |   16(0x10)    | --          |
| ^             | ^             | ^                                               | Read/Write Multiple Registers    |   23(0x17)    | --          |
| ^             | ^             | ^                                               | Mask Write Register              |   22(0x16)    | --          |
| ^             | ^             | ^                                               | Read FIFO Queue                  |   24(0x18)    | --          |
| ^             | ^             | File Record Access                              | Read File Record                 |   20(0x14)    | --          |
| ^             | ^             | ^                                               | Write File Record                |   21(0x15)    | --          |
| Diagnostics   |               |                                                 | Read Exception Status            |    7(0x07)    | serial only |
| ^             | ^             | ^                                               | Diagnostic                       |    8(0x08)    | serial only |
| ^             | ^             | ^                                               | Get Com Event Counter            |   11(0x0B)    | serial only |
| ^             | ^             | ^                                               | Get Com Event Log                |   12(0x0C)    | serial only |
| ^             | ^             | ^                                               | Report Server ID                 |   17(0x11)    | serial only |
| ^             | ^             | ^                                               | Read Device Identification       |   43(0x2B)    | --          |
| Other         |               |                                                 | Encapsulated Interface Transport |   43(0x2B)    | --          |
    */
    int length = 0;
    switch (frame.function) {
        ///////////////////////////////////
        // Diagnostics
        ///////////////////////////////////
        case 0x07: // read_exception_status (serial line only)
            length            = this->_exception_queue.length();
            frame.data_length = length;
            for (int i = length; i > 0; i--) {
                frame.data[i - 1] = this->_exception_queue.pop();
            }
            break;
        case 0x08: // diagnostics (serial line only)
            break;
        case 0x0b: // get_comm_event_counter (serial line only)
            break;
        case 0x0c: // get_comm_event_log (serial line only)
            break;
        case 0x11: // report_server_id (serial line only)
            break;
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - Internal Registers or Physical Output Registers
        ///////////////////////////////////
        case 0x03: // read_holding_registers
            if (true == this->_read_multiple_holding_registers(frame)) {
                this->count_process++;
            }
            break;
        case 0x06: // write_single_register
            if (true == this->_write_single_register(frame)) {
                this->count_process++;
            }
            break;
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - Internal Registers or Physical Output Registers
        ///////////////////////////////////
        case 0x10: // write_multiple_registers
            if (true == this->_write_multiple_registers(frame)) {
                this->count_process++;
            }
            break;
        case 0x17: // read/write_multiple_registers
        case 0x16: // mask_write_register
        case 0x18: // read_fifo_queue
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - Physical Discrete Inputs
        ///////////////////////////////////
        case 0x04: // read_input_registers
        ///////////////////////////////////
        // Data Access
        // - 16-bit access
        //   - File Record Access
        ///////////////////////////////////
        case 0x14: // read_file_record
        case 0x15: // write_file_record
        ///////////////////////////////////
        // Data Access
        // - Bit access
        //   - Internal Bits or Physical Coils
        ///////////////////////////////////
        case 0x01: // read_coils
        case 0x05: // write_single_coil
        case 0x0f: // write_multiple_coils
        ///////////////////////////////////
        // Other
        ///////////////////////////////////
        case 0x2b: // can_open_general reference request and response
        ///////////////////////////////////
        // Data Access
        // - Bit access
        //   - Physical Discrete Inputs
        ///////////////////////////////////
        case 0x02: // read_discrete_inputs
        default:
            this->_exception_queue.push(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
            frame.happened_error(MessageFrame::EXCEPTION_CODE::CODE_ILLEGAL_FUNCTION);
            break;
    }
    if (count_process > 0xFFFFFFF) {
        count_process = 0;
    }
    if (count_reception > 0xFFFFFFF) {
        count_process   = 0;
        count_reception = 0;
    }
    return frame;
}

// =============================
// PUBLIC : Function : getter/setter
// =============================
void WheelController::set_accel(float x, float y, float z)
{
    this->accel.x = (uint16_t)(x * 1000);
    this->accel.y = (uint16_t)(y * 1000);
    this->accel.z = (uint16_t)(z * 1000);
}
void WheelController::set_gyro(float x, float y, float z)
{
    this->gyro.x = (uint16_t)(x * 1000);
    this->gyro.y = (uint16_t)(y * 1000);
    this->gyro.z = (uint16_t)(z * 1000);
}
bool WheelController::set_pwm_servo(int index, int value)
{
    bool result = false;
    if ((0 <= index) && (index < this->pwm_servo_count)) {
        this->pwm_servo_request[index] = value;
        result                         = true;
    }
    return result;
}
uint32_t WheelController::get_oscillator_frequency(void)
{
    return this->_oscillator_frequency;
}
uint32_t WheelController::get_pwm_freq(void)
{
    return this->_pwm_freq;
}

// =============================
// PRIVATE : Function : MODBUS
// =============================
bool WheelController::_read_multiple_holding_registers(MessageFrame &frame)
{
    bool result     = false;
    int data_length = (frame.data[2] << 8) | frame.data[3];
    int array_index = 2;
    if (0x31 == frame.data[0]) {
        array_index = this->_response_pwm_servo_actual(frame, frame.data[1], data_length, array_index);
        result      = true;
    } else if (0x68 == frame.data[0]) {
        array_index = this->_response_accel_and_gyro(frame, frame.data[1], data_length, array_index);
        result      = true;
    } else if (0x70 == frame.data[0]) {
        array_index = this->_response_pwm_servo_setting(frame, frame.data[1], data_length, array_index);
        result      = true;
    } else if (0x71 == frame.data[0]) {
        array_index = this->_response_pwm_servo_request(frame, frame.data[1], data_length, array_index);
        result      = true;
    }
    frame.data_length = array_index;
    data_length       = (array_index - 2) / 2;
    if (data_length < 0) {
        data_length = 0;
    }

    frame.data[0] = (data_length >> 8) & 0xFF;
    frame.data[1] = data_length & 0xFF;
    return result;
}
bool WheelController::_write_single_register(MessageFrame &frame)
{
    bool result    = false;
    int length     = 0;
    uint32_t value = 0;
    if (0x71 == frame.data[0]) {
        result = true;
        value  = (frame.data[2] << 8) | (frame.data[3]);
        if (0x00 == frame.data[1]) {
            for (int i = 0; i < this->pwm_servo_count; i++) {
                this->set_pwm_servo(i, (int)(value));
            }
            length++;
        } else {
            this->set_pwm_servo((int)frame.data[1] - 1, (int)(value));
            length++;
        }
    }
    if (0x70 == frame.data[0]) {
        bool flag_osc = false;
        bool flag_pwm = false;
        value         = (frame.data[2] << 8) | (frame.data[3]);
        if (0x00 == frame.data[1]) {
            this->_oscillator_frequency = (uint32_t)((value << 16) | (this->_oscillator_frequency & 0xFFFF));
            flag_osc                    = true;
        }
        if (0x01 == frame.data[1]) {
            this->_oscillator_frequency = (uint32_t)((this->_oscillator_frequency & 0xFFFF0000) | (value & 0xFFFF));
            flag_osc                    = true;
        }
        if (0x02 == frame.data[1]) {
            this->_pwm_freq = (uint32_t)((value << 16) | (this->_pwm_freq & 0xFFFF));
            flag_pwm        = true;
        }
        if (0x03 == frame.data[1]) {
            this->_pwm_freq = (uint32_t)((this->_pwm_freq & 0xFFFF0000) | (value & 0xFFFF));
            flag_pwm        = true;
        }
        if (true == flag_osc) {
            this->_pwm->setOscillatorFrequency(this->_oscillator_frequency);
            result = true;
            length++;
        }
        if (true == flag_pwm) {
            this->_pwm->setPWMFreq(this->_pwm_freq);
            result = true;
            length++;
        }
        if ((true == flag_osc) || (true == flag_pwm)) {
            this->_save_setting_setting();
        }
    }
    frame.data_length = 2;
    frame.data[0]     = (length >> 8) & 0xFF;
    frame.data[1]     = length & 0xFF;
    return result;
}
bool WheelController::_write_multiple_registers(MessageFrame &frame)
{
    bool result     = false;
    int length      = 0;
    int loot_length = 0;
    if (0x71 == frame.data[0]) {
        int count = 4;

        result = true;
        if (0x00 == frame.data[1]) {
            frame.data[1] = 0x01;
            loot_length   = this->pwm_servo_count + frame.data[1];
            for (int i = frame.data[1]; (i <= this->pwm_servo_count) && (i < loot_length); i++) {
                frame.data[count++] = frame.data[4];
                frame.data[count++] = frame.data[5];
            }
            count = 4;
        } else {
            loot_length = ((frame.data[2] << 8) | frame.data[3]) + frame.data[1];
        }
        for (int i = frame.data[1]; (i <= this->pwm_servo_count) && (i < loot_length); i++) {
            this->set_pwm_servo(i - 1, (int)((frame.data[count++] << 8) | frame.data[count++]));
            length++;
        }
    }
    if (0x70 == frame.data[0]) {
        int count                     = 4;
        bool flag_osc                 = false;
        bool flag_pwm                 = false;
        bool flag_write               = false;
        result                        = true;
        uint32_t oscillator_frequency = this->_oscillator_frequency;
        uint32_t pwm_freq             = this->_pwm_freq;
        uint32_t value                = 0;
        int data_length               = ((frame.data[2] << 8) | frame.data[3]) + length;
        /////////////////////////////////
        if ((0x00 == frame.data[1]) || ((true == flag_write) && (length < data_length))) {
            value                = (frame.data[count++] << 8) | (frame.data[count++]);
            oscillator_frequency = (uint32_t)((value << 16) | (oscillator_frequency & 0xFFFF));
            flag_osc             = true;
            flag_write           = true;
            length++;
        }
        if ((0x01 == frame.data[1]) || ((true == flag_write) && (length < data_length))) {
            value                = (frame.data[count++] << 8) | (frame.data[count++]);
            oscillator_frequency = (uint32_t)((oscillator_frequency & 0xFFFF0000) | (value & 0xFFFF));
            flag_osc             = true;
            flag_write           = true;
            length++;
        }
        if ((0x02 == frame.data[1]) || ((true == flag_write) && (length < data_length))) {
            value      = (frame.data[count++] << 8) | (frame.data[count++]);
            pwm_freq   = (uint32_t)((value << 16) | (pwm_freq & 0xFFFF));
            flag_pwm   = true;
            flag_write = true;
            length++;
        }
        if ((0x03 == frame.data[1]) || ((true == flag_write) && (length < data_length))) {
            value      = (frame.data[count++] << 8) | (frame.data[count++]);
            pwm_freq   = (uint32_t)((pwm_freq & 0xFFFF0000) | (value & 0xFFFF));
            flag_pwm   = true;
            flag_write = true;
            length++;
        }

        /////////////////////////////////
        if (true == flag_osc) {
            this->_oscillator_frequency = oscillator_frequency;
            this->_pwm->setOscillatorFrequency(this->_oscillator_frequency);
            result = true;
        }
        if (true == flag_pwm) {
            this->_pwm_freq = pwm_freq;
            this->_pwm->setPWMFreq(this->_pwm_freq);
            result = true;
        }
        if ((true == flag_osc) || (true == flag_pwm)) {
            this->_save_setting_setting();
        }
    }
    frame.data_length = 2;
    frame.data[0]     = (length >> 8) & 0xFF;
    frame.data[1]     = length & 0xFF;
    return result;
}

// =============================
// PRIVATE : Function : MODBUS : Response
// =============================
int WheelController::_response_accel_and_gyro(MessageFrame &frame, unsigned int sub_address, int length, int start_index)
{
    bool is_skip = true;
    /*
        | Num    | Device type | --      | Note |
        | ------ | ----------- | ------- | ---- |
        | 0x6810 | IMU         | accel.x | --   |
        | 0x6811 | IMU         | accel.y | --   |
        | 0x6812 | IMU         | accel.z | --   |
        | 0x6813 | IMU         | gyro.x  | --   |
        | 0x6814 | IMU         | gyro.y  | --   |
        | 0x6815 | IMU         | gyro.z  | --   |
    */
    if ((0 < length)) {
        // accel
        if ((0x10 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->accel.x >> 8) & 0xFF;
            frame.data[start_index++] = (this->accel.x >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x11 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->accel.y >> 8) & 0xFF;
            frame.data[start_index++] = (this->accel.y >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x12 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->accel.z >> 8) & 0xFF;
            frame.data[start_index++] = (this->accel.z >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        // gyro
        if ((0x13 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->gyro.x >> 8) & 0xFF;
            frame.data[start_index++] = (this->gyro.x >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x14 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->gyro.y >> 8) & 0xFF;
            frame.data[start_index++] = (this->gyro.y >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x15 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->gyro.z >> 8) & 0xFF;
            frame.data[start_index++] = (this->gyro.z >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if (true == is_skip) {
            frame.data[start_index]     = 0x00;
            frame.data[start_index + 1] = 0x00;
        }
        for (int i = length; i > 0; i--) {
            frame.data[start_index++] = 0x00;
            frame.data[start_index++] = 0x00;
        }
    }

    return start_index;
}
int WheelController::_response_pwm_servo_actual(MessageFrame &frame, unsigned int sub_address, int length, int start_index)
{
    bool is_skip = true;
    /*
    | Num    | Device type            | --  | Note |
    | ------ | ---------------------- | --- | ---- |
    | 0x3101 | PWM Motor(I2C) - No.01 | --  |      |
    | 0x3102 | PWM Motor(I2C) - No.02 | --  |      |
    | 0x3103 | PWM Motor(I2C) - No.03 | --  |      |
    | ...    |                        |     |      |
    */
    if ((0 < length)) {
        if (0x00 == sub_address) {
            is_skip = false;
        }
        for (int i = 0; i < this->pwm_servo_count; i++) {
            if (((i + 1) == sub_address) || ((false == is_skip) && (0 < length))) {
                frame.data[start_index++] = (this->pwm_servo[i] >> 8) & 0xFF;
                frame.data[start_index++] = (this->pwm_servo[i] >> 0) & 0xFF;
                is_skip                   = false;
            }
            length--;
        }
        if (true == is_skip) {
            frame.data[start_index]     = 0x00;
            frame.data[start_index + 1] = 0x00;
        }
        for (int i = length; i > 0; i--) {
            frame.data[start_index++] = 0x00;
            frame.data[start_index++] = 0x00;
        }
    }
    return start_index;
}
int WheelController::_response_pwm_servo_request(MessageFrame &frame, unsigned int sub_address, int length, int start_index)
{
    bool is_skip = true;
    /*
        | Num    | Device type            | --  | Note          |
        | ------ | ---------------------- | --- | ------------- |
        | 0x7100 | PWM Motor(I2C) - ALL   | --  | for Broadcast |
        | 0x7101 | PWM Motor(I2C) - No.01 | --  |               |
        | 0x7102 | PWM Motor(I2C) - No.02 | --  |               |
        | 0x7103 | PWM Motor(I2C) - No.03 | --  |               |
        | 0x7104 | PWM Motor(I2C) - No.04 | --  |               |
        | 0x7105 | PWM Motor(I2C) - No.05 | --  |               |
    */
    if ((0 < length)) {
        if (0x00 == sub_address) {
            is_skip = false;
        }
        for (int i = 0; i < this->pwm_servo_count; i++) {
            if (((i + 1) == sub_address) || ((false == is_skip) && (0 < length))) {
                frame.data[start_index++] = (this->pwm_servo_request[i] >> 8) & 0xFF;
                frame.data[start_index++] = (this->pwm_servo_request[i] >> 0) & 0xFF;
                is_skip                   = false;
                length--;
            }
        }
        if (true == is_skip) {
            frame.data[start_index]     = 0x00;
            frame.data[start_index + 1] = 0x00;
        }
        for (int i = length; i > 0; i--) {
            frame.data[start_index++] = 0x00;
            frame.data[start_index++] = 0x00;
        }
    }
    return start_index;
}
int WheelController::_response_pwm_servo_setting(MessageFrame &frame, unsigned int sub_address, int length, int start_index)
{
    bool is_skip = true;
    /*
        | Num    | Device type              | --                      | Note |
        | ------ | ------------------------ | ----------------------- | ---- |
        | 0x7000 | PWM Motor(I2C) - Setting | Oscillator Frequency(H) |      |
        | 0x7001 | PWM Motor(I2C) - Setting | Oscillator Frequency(L) |      |
        | 0x7002 | PWM Motor(I2C) - Setting | PWM Frequency(H)        |      |
        | 0x7003 | PWM Motor(I2C) - Setting | PWM Frequency(L)        |      |
    */
    if ((0 < length)) {
        // accel
        if ((0x00 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->_oscillator_frequency >> 24) & 0xFF;
            frame.data[start_index++] = (this->_oscillator_frequency >> 16) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x01 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->_oscillator_frequency >> 8) & 0xFF;
            frame.data[start_index++] = (this->_oscillator_frequency >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x02 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->_pwm_freq >> 24) & 0xFF;
            frame.data[start_index++] = (this->_pwm_freq >> 16) & 0xFF;
            length--;
            is_skip = false;
        }
        if ((0x03 == sub_address) || ((false == is_skip) && (0 < length))) {
            frame.data[start_index++] = (this->_pwm_freq >> 8) & 0xFF;
            frame.data[start_index++] = (this->_pwm_freq >> 0) & 0xFF;
            length--;
            is_skip = false;
        }
        if (true == is_skip) {
            frame.data[start_index]     = 0x00;
            frame.data[start_index + 1] = 0x00;
        }
        for (int i = length; i > 0; i--) {
            frame.data[start_index++] = 0x00;
            frame.data[start_index++] = 0x00;
        }
    }

    return start_index;
}

// =============================
// PRIVATE : Function : File
// =============================
bool WheelController::_load_setting_setting()
{
    bool result = false;
    try {
        if (SPIFFS.begin(true)) {
            if (false == SPIFFS.exists(this->_setting_file_name)) {
                result = this->_save_setting_setting(SPIFFS);
            } else {
                result = true;
            }
            if (true == result) {
                result = this->_load_setting_setting(SPIFFS);
            }
            SPIFFS.end();
        }
    } catch (...) {
        result = false;
    }

    return result;
}

bool WheelController::_load_setting_setting(fs::FS &fs)
{
    bool result = false;
    if (true == fs.exists(this->_setting_file_name)) {
        File dataFile = fs.open(this->_setting_file_name, FILE_READ);
        if (!dataFile) {
            result = false;
        } else {
            bool flag_break = false;
            result          = true;

            while (dataFile.available()) {
                String word = dataFile.readStringUntil('\n');
                word.replace("\r", "");
                word.replace("\n", "");
                int index_start = 0;
                int next        = word.indexOf(':');
                String address  = word.substring(index_start, next);

                index_start        = next + 1;
                next               = word.indexOf(':', index_start);
                String data_length = word.substring(index_start, next);

                index_start  = next + 1;
                String value = word.substring(index_start);

                if (0 < word.length()) {
                    if (true == address.equals("0x7000")) {
                        if (0 < value.length()) {
                            this->_oscillator_frequency = value.toInt();
                        }
                    }
                    if (true == address.equals("0x7002")) {
                        if (0 < value.length()) {
                            this->_pwm_freq = value.toInt();
                        }
                    }
                }
            }
            dataFile.close();
        }
    }
    return result;
}
bool WheelController::_save_setting_setting()
{
    bool result = false;
    if (SPIFFS.begin(true)) {
        result = this->_save_setting_setting(SPIFFS);
        SPIFFS.end();
    }

    return result;
}
bool WheelController::_save_setting_setting(fs::FS &fs)
{
    bool result   = false;
    File dataFile = fs.open(this->_setting_file_name, FILE_WRITE);
    dataFile.printf("0x7000:2:%d\n", this->_oscillator_frequency);
    dataFile.printf("0x7002:2:%d\n", this->_pwm_freq);
    dataFile.close();
    result = true;

    return result;
}
