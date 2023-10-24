/**
 * @file cmd_roboclaw_list.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-01-15
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef CMD_ROBOCLAW_LIST_HPP
#define CMD_ROBOCLAW_LIST_HPP

class CmdRoboclawData {
public:
    int Id           = 0;
    int Size_send    = 0;
    int Size_receive = 0;
    CmdRoboclawData(int id = 0, int send = 0, int receive = 0)
    {
        this->Id           = id;
        this->Size_send    = send;
        this->Size_receive = receive;
    }
};

CmdRoboclawData CmdRoboclawDataList[0x100] = {
    CmdRoboclawData(0x00, 5, 1),    // Drive Forward Motor 1
    CmdRoboclawData(0x01, 5, 1),    // Drive Backwards Motor 1
    CmdRoboclawData(0x02, 5, 1),    // Set Main Voltage Minimum
    CmdRoboclawData(0x03, 5, 1),    // Set Main Voltage Maximum
    CmdRoboclawData(0x04, 5, 1),    // Drive Forward Motor 2
    CmdRoboclawData(0x05, 5, 1),    // Drive Backwards Motor 2
    CmdRoboclawData(0x06, 5, 1),    // Drive Motor 1 (7 Bit)
    CmdRoboclawData(0x07, 5, 1),    // Drive Motor 2 (7 Bit)
    CmdRoboclawData(0x08, 5, 1),    // Drive Forward Mixed Mode
    CmdRoboclawData(0x09, 5, 1),    // Drive Backwards Mixed Mode
    CmdRoboclawData(0x0A, 5, 1),    // Turn Right Mixed Mode
    CmdRoboclawData(0x0B, 5, 1),    // Turn Left Mixed Mode
    CmdRoboclawData(0x0C, 5, 1),    // Drive Forward or Backward (7 bit)
    CmdRoboclawData(0x0D, 5, 1),    // Turn Left or Right (7 Bit)
    CmdRoboclawData(0x0E, 5, 1),    // Set Serial Timeout
    CmdRoboclawData(0x0F, 5, 1),    // Read Serial Timeout
    CmdRoboclawData(0x10, 2, 7),    // Read Encoder Count/Value for M1.
    CmdRoboclawData(0x11, 2, 7),    // Read Encoder Count/Value for M2.
    CmdRoboclawData(0x12, 2, 7),    // Read M1 Speed in Encoder Counts Per Second.
    CmdRoboclawData(0x13, 2, 7),    // Read M2 Speed in Encoder Counts Per Second.
    CmdRoboclawData(0x14, 4, 1),    // Resets Encoder Registers for M1 and M2(Quadrature only).
    CmdRoboclawData(0x15, 2, 0xFF), // Read Firmware Version
    CmdRoboclawData(0x16, 8, 1),    // Set Encoder 1 Register(Quadrature only).
    CmdRoboclawData(0x17, 8, 1),    // Set Encoder 2 Register(Quadrature only).
    CmdRoboclawData(0x18, 2, 4),    // Read Main Battery Voltage
    CmdRoboclawData(0x19, 2, 4),    // Read Logic Battery Voltage
    CmdRoboclawData(0x1A, 5, 1),    // Set Minimum Logic Voltage Level
    CmdRoboclawData(0x1B, 5, 1),    // Set Maximum Logic Voltage Level
    CmdRoboclawData(0x1C, 20, 1),   // Set Velocity PID Constants for M1.
    CmdRoboclawData(0x1D, 20, 1),   // Set Velocity PID Constants for M2.
    CmdRoboclawData(0x1E, 2, 7),    // Read Current M1 Raw Speed
    CmdRoboclawData(0x1F, 2, 7),    // Read Current M2 Raw Speed
    CmdRoboclawData(0x20, 6, 1),    // Drive M1 With Signed Duty Cycle. (Encoders not required)
    CmdRoboclawData(0x21, 6, 1),    // Drive M2 With Signed Duty Cycle. (Encoders not required)
    CmdRoboclawData(0x22, 8, 1),    // Drive M1 / M2 With Signed Duty Cycle. (Encoders not required)
    CmdRoboclawData(0x23, 8, 1),    // Drive M1 With Signed Speed.
    CmdRoboclawData(0x24, 8, 1),    // Drive M2 With Signed Speed.
    CmdRoboclawData(0x25, 12, 1),   // Drive M1 / M2 With Signed Speed.
    CmdRoboclawData(0x26, 12, 1),   // Drive M1 With Signed Speed And Acceleration.
    CmdRoboclawData(0x27, 12, 1),   // Drive M2 With Signed Speed And Acceleration.
    CmdRoboclawData(0x28, 16, 1),   // Drive M1 / M2 With Signed Speed And Acceleration.
    CmdRoboclawData(0x29, 13, 1),   // Drive M1 With Signed Speed And Distance. Buffered.
    CmdRoboclawData(0x2A, 13, 1),   // Drive M2 With Signed Speed And Distance. Buffered.
    CmdRoboclawData(0x2B, 21, 1),   // Drive M1 / M2 With Signed Speed And Distance. Buffered.
    CmdRoboclawData(0x2C, 17, 1),   // Drive M1 With Signed Speed, Acceleration and Distance. Buffered.
    CmdRoboclawData(0x2D, 17, 1),   // Drive M2 With Signed Speed, Acceleration and Distance. Buffered.
    CmdRoboclawData(0x2E, 25, 1),   // Drive M1 / M2 With Signed Speed, Acceleration And Distance. Buffered.
    CmdRoboclawData(0x2F, 2, 4),    // Read Buffer Length.
    CmdRoboclawData(0x30, 2, 6),    // Read Motor PWMs
    CmdRoboclawData(0x31, 2, 6),    // Read Motor Currents
    CmdRoboclawData(0x32, 20, 1),   // Drive M1 / M2 With Individual Signed Speed and Acceleration
    CmdRoboclawData(0x33, 29, 1),   // Drive M1 / M2 With Individual Signed Speed, Accel and Distance
    CmdRoboclawData(0x34, 8, 1),    // Drive M1 With Signed Duty and Accel. (Encoders not required)
    CmdRoboclawData(0x35, 8, 1),    // Drive M2 With Signed Duty and Accel. (Encoders not required)
    CmdRoboclawData(0x36, 16, 1),   // Drive M1 / M2 With Signed Duty and Accel. (Encoders not required)
    CmdRoboclawData(0x37, 2, 18),   // Read Motor 1 Velocity PID Constants
    CmdRoboclawData(0x38, 2, 18),   // Read Motor 2 Velocity PID Constants
    CmdRoboclawData(0x39, 8, 1),    // Set Main Battery Voltages
    CmdRoboclawData(0x3A, 8, 1),    // Set Logic Battery Voltages
    CmdRoboclawData(0x3B, 2, 6),    // Read Main Battery Voltage Settings
    CmdRoboclawData(0x3C, 2, 6),    // Read Logic Battery Voltage Settings
    CmdRoboclawData(0x3D, 32, 1),   // Set Position PID Constants for M1.
    CmdRoboclawData(0x3E, 32, 1),   // Set Position PID Constants for M2
    CmdRoboclawData(0x3F, 2, 30),   // Read Motor 1 Position PID Constants
    CmdRoboclawData(0x40, 2, 30),   // Read Motor 2 Position PID Constants
    CmdRoboclawData(0x41, 21, 1),   // Drive M1 with Speed, Accel, Deccel and Position
    CmdRoboclawData(0x42, 21, 1),   // Drive M2 with Speed, Accel, Deccel and Position
    CmdRoboclawData(0x43, 37, 1),   // Drive M1 / M2 with Speed, Accel, Deccel and Position
    CmdRoboclawData(0x44, 8, 1),    // Set default duty cycle acceleration for M1
    CmdRoboclawData(0x45, 8, 1),    // Set default duty cycle acceleration for M2
    CmdRoboclawData(0x46, 6, 1),    // Set Default Speed for M1
    CmdRoboclawData(0x47, 6, 1),    // Set Default Speed for M2
    CmdRoboclawData(0x48, 2, 6),    // Read Default Speed Settings
    CmdRoboclawData(0x49, 0, 0),    //
    CmdRoboclawData(0x4A, 7, 1),    // Set S3,S4 and S5 Modes
    CmdRoboclawData(0x4B, 2, 5),    // Read S3,S4 and S5 Modes
    CmdRoboclawData(0x4C, 6, 1),    // Set DeadBand for RC/Analog controls
    CmdRoboclawData(0x4D, 2, 4),    // Read DeadBand for RC/Analog controls
    CmdRoboclawData(0x4E, 2, 10),   // Read Encoders Counts
    CmdRoboclawData(0x4F, 2, 10),   // Read Raw Motor Speeds
    CmdRoboclawData(0x50, 2, 1),    // Restore Defaults
    CmdRoboclawData(0x51, 2, 10),   // Read Default Duty Cycle Accelerations
    CmdRoboclawData(0x52, 2, 4),    // Read Temperature
    CmdRoboclawData(0x53, 2, 4),    // Read Temperature 2
    CmdRoboclawData(0x54, 0, 0),    //
    CmdRoboclawData(0x55, 0, 0),    //
    CmdRoboclawData(0x56, 0, 0),    //
    CmdRoboclawData(0x57, 0, 0),    //
    CmdRoboclawData(0x58, 0, 0),    //
    CmdRoboclawData(0x59, 0, 0),    //
    CmdRoboclawData(0x5A, 2, 3),    // Read Status
    CmdRoboclawData(0x5B, 2, 4),    // Read Encoder Modes
    CmdRoboclawData(0x5C, 5, 1),    // Set Motor 1 Encoder Mode
    CmdRoboclawData(0x5D, 5, 1),    // Set Motor 2 Encoder Mode
    CmdRoboclawData(0x5E, 2, 1),    // Write Settings to EEPROM
    CmdRoboclawData(0x5F, 2, 4),    // Read Settings from EEPROM
    CmdRoboclawData(0x60, 0, 0),    //
    CmdRoboclawData(0x61, 0, 0),    //
    CmdRoboclawData(0x62, 6, 1),    // Set Standard Config Settings
    CmdRoboclawData(0x63, 2, 4),    // Read Standard Config Settings
    CmdRoboclawData(0x64, 4, 1),    // Set CTRL Modes
    CmdRoboclawData(0x65, 2, 4),    // Read CTRL Modes
    CmdRoboclawData(0x66, 6, 1),    // Set CTRL1
    CmdRoboclawData(0x67, 6, 1),    // Set CTRL2
    CmdRoboclawData(0x68, 2, 6),    // Read CTRLs
    CmdRoboclawData(0x69, 10, 1),   // Set Auto Home Duty/Speed and Timeout M1
    CmdRoboclawData(0x6A, 10, 1),   // Set Auto Home Duty/Speed and Timeout M2
    CmdRoboclawData(0x6B, 2, 8),    // Read Auto Home Settings
    CmdRoboclawData(0x6C, 2, 10),   // Read Motor Average Speeds
    CmdRoboclawData(0x6D, 12, 1),   // Set Speed Error Limits
    CmdRoboclawData(0x6E, 2, 10),   // Read Speed Error Limits
    CmdRoboclawData(0x6F, 2, 10),   // Read Speed Errors
    CmdRoboclawData(0x70, 12, 1),   // Set Position Error Limits
    CmdRoboclawData(0x71, 2, 10),   // Read Position Error Limits
    CmdRoboclawData(0x72, 2, 10),   // Read Position Errors
    CmdRoboclawData(0x73, 6, 1),    // Set Battery Voltage Offsets
    CmdRoboclawData(0x74, 2, 4),    // Read Battery Voltage Offsets
    CmdRoboclawData(0x75, 8, 1),    // Set Current Blanking Percentages
    CmdRoboclawData(0x76, 2, 6),    // Read Current Blanking Percentages
    CmdRoboclawData(0x77, 9, 1),    // Drive M1 with Position.
    CmdRoboclawData(0x78, 9, 1),    // Drive M2 with Position.
    CmdRoboclawData(0x79, 13, 1),   // Drive M1/M2 with Position.
    CmdRoboclawData(0x7A, 13, 1),   // Drive M1 with Speed and Position.
    CmdRoboclawData(0x7B, 13, 1),   // Drive M2 with Speed and Position.
    CmdRoboclawData(0x7C, 13, 1),   // Drive M1/M2 with Speed and Position.
    CmdRoboclawData(0x7D, 0, 0),    //
    CmdRoboclawData(0x7E, 0, 0),    //
    CmdRoboclawData(0x7F, 0, 0),    //
    CmdRoboclawData(0x80, 0, 0),    //
    CmdRoboclawData(0x81, 0, 0),    //
    CmdRoboclawData(0x82, 0, 0),    //
    CmdRoboclawData(0x83, 0, 0),    //
    CmdRoboclawData(0x84, 0, 0),    //
    CmdRoboclawData(0x85, 12, 1),   // Set M1 Maximum Current
    CmdRoboclawData(0x86, 12, 1),   // Set M2 Maximum Current
    CmdRoboclawData(0x87, 2, 10),   // Read M1 Maximum Current
    CmdRoboclawData(0x88, 2, 10),   // Read M2 Maximum Current
    CmdRoboclawData(0x89, 0, 0),    //
    CmdRoboclawData(0x8A, 0, 0),    //
    CmdRoboclawData(0x8B, 0, 0),    //
    CmdRoboclawData(0x8C, 0, 0),    //
    CmdRoboclawData(0x8D, 0, 0),    //
    CmdRoboclawData(0x8E, 0, 0),    //
    CmdRoboclawData(0x8F, 0, 0),    //
    CmdRoboclawData(0x90, 0, 0),    //
    CmdRoboclawData(0x91, 0, 0),    //
    CmdRoboclawData(0x92, 0, 0),    //
    CmdRoboclawData(0x93, 0, 0),    //
    CmdRoboclawData(0x94, 5, 1),    // Set PWM Mode
    CmdRoboclawData(0x95, 2, 3),    // Read PWM Mode
    CmdRoboclawData(0x96, 0, 0),    //
    CmdRoboclawData(0x97, 0, 0),    //
    CmdRoboclawData(0x98, 0, 0),    //
    CmdRoboclawData(0x99, 0, 0),    //
    CmdRoboclawData(0x9A, 0, 0),    //
    CmdRoboclawData(0x9B, 0, 0),    //
    CmdRoboclawData(0x9C, 0, 0),    //
    CmdRoboclawData(0x9D, 0, 0),    //
    CmdRoboclawData(0x9E, 0, 0),    //
    CmdRoboclawData(0x9F, 0, 0),    //
    CmdRoboclawData(0xA0, 0, 0),    //
    CmdRoboclawData(0xA1, 0, 0),    //
    CmdRoboclawData(0xA2, 0, 0),    //
    CmdRoboclawData(0xA3, 0, 0),    //
    CmdRoboclawData(0xA4, 0, 0),    //
    CmdRoboclawData(0xA5, 0, 0),    //
    CmdRoboclawData(0xA6, 0, 0),    //
    CmdRoboclawData(0xA7, 0, 0),    //
    CmdRoboclawData(0xA8, 0, 0),    //
    CmdRoboclawData(0xA9, 0, 0),    //
    CmdRoboclawData(0xAA, 0, 0),    //
    CmdRoboclawData(0xAB, 0, 0),    //
    CmdRoboclawData(0xAC, 0, 0),    //
    CmdRoboclawData(0xAD, 0, 0),    //
    CmdRoboclawData(0xAE, 0, 0),    //
    CmdRoboclawData(0xAF, 0, 0),    //
    CmdRoboclawData(0xB0, 0, 0),    //
    CmdRoboclawData(0xB1, 0, 0),    //
    CmdRoboclawData(0xB2, 0, 0),    //
    CmdRoboclawData(0xB3, 0, 0),    //
    CmdRoboclawData(0xB4, 0, 0),    //
    CmdRoboclawData(0xB5, 0, 0),    //
    CmdRoboclawData(0xB6, 0, 0),    //
    CmdRoboclawData(0xB7, 0, 0),    //
    CmdRoboclawData(0xB8, 0, 0),    //
    CmdRoboclawData(0xB9, 0, 0),    //
    CmdRoboclawData(0xBA, 0, 0),    //
    CmdRoboclawData(0xBB, 0, 0),    //
    CmdRoboclawData(0xBC, 0, 0),    //
    CmdRoboclawData(0xBD, 0, 0),    //
    CmdRoboclawData(0xBE, 0, 0),    //
    CmdRoboclawData(0xBF, 0, 0),    //
    CmdRoboclawData(0xC0, 0, 0),    //
    CmdRoboclawData(0xC1, 0, 0),    //
    CmdRoboclawData(0xC2, 0, 0),    //
    CmdRoboclawData(0xC3, 0, 0),    //
    CmdRoboclawData(0xC4, 0, 0),    //
    CmdRoboclawData(0xC5, 0, 0),    //
    CmdRoboclawData(0xC6, 0, 0),    //
    CmdRoboclawData(0xC7, 0, 0),    //
    CmdRoboclawData(0xC8, 0, 0),    //
    CmdRoboclawData(0xC9, 0, 0),    //
    CmdRoboclawData(0xCA, 0, 0),    //
    CmdRoboclawData(0xCB, 0, 0),    //
    CmdRoboclawData(0xCC, 0, 0),    //
    CmdRoboclawData(0xCD, 0, 0),    //
    CmdRoboclawData(0xCE, 0, 0),    //
    CmdRoboclawData(0xCF, 0, 0),    //
    CmdRoboclawData(0xD0, 0, 0),    //
    CmdRoboclawData(0xD1, 0, 0),    //
    CmdRoboclawData(0xD2, 0, 0),    //
    CmdRoboclawData(0xD3, 0, 0),    //
    CmdRoboclawData(0xD4, 0, 0),    //
    CmdRoboclawData(0xD5, 0, 0),    //
    CmdRoboclawData(0xD6, 0, 0),    //
    CmdRoboclawData(0xD7, 0, 0),    //
    CmdRoboclawData(0xD8, 0, 0),    //
    CmdRoboclawData(0xD9, 0, 0),    //
    CmdRoboclawData(0xDA, 0, 0),    //
    CmdRoboclawData(0xDB, 0, 0),    //
    CmdRoboclawData(0xDC, 0, 0),    //
    CmdRoboclawData(0xDD, 0, 0),    //
    CmdRoboclawData(0xDE, 0, 0),    //
    CmdRoboclawData(0xDF, 0, 0),    //
    CmdRoboclawData(0xE0, 0, 0),    //
    CmdRoboclawData(0xE1, 0, 0),    //
    CmdRoboclawData(0xE2, 0, 0),    //
    CmdRoboclawData(0xE3, 0, 0),    //
    CmdRoboclawData(0xE4, 0, 0),    //
    CmdRoboclawData(0xE5, 0, 0),    //
    CmdRoboclawData(0xE6, 0, 0),    //
    CmdRoboclawData(0xE7, 0, 0),    //
    CmdRoboclawData(0xE8, 0, 0),    //
    CmdRoboclawData(0xE9, 0, 0),    //
    CmdRoboclawData(0xEA, 0, 0),    //
    CmdRoboclawData(0xEB, 0, 0),    //
    CmdRoboclawData(0xEC, 0, 0),    //
    CmdRoboclawData(0xED, 0, 0),    //
    CmdRoboclawData(0xEE, 0, 0),    //
    CmdRoboclawData(0xEF, 0, 0),    //
    CmdRoboclawData(0xF0, 0, 0),    //
    CmdRoboclawData(0xF1, 0, 0),    //
    CmdRoboclawData(0xF2, 0, 0),    //
    CmdRoboclawData(0xF3, 0, 0),    //
    CmdRoboclawData(0xF4, 0, 0),    //
    CmdRoboclawData(0xF5, 0, 0),    //
    CmdRoboclawData(0xF6, 0, 0),    //
    CmdRoboclawData(0xF7, 0, 0),    //
    CmdRoboclawData(0xF8, 0, 0),    //
    CmdRoboclawData(0xF9, 0, 0),    //
    CmdRoboclawData(0xFA, 0, 0),    //
    CmdRoboclawData(0xFB, 0, 0),    //
    CmdRoboclawData(0xFC, 7, 1),    // Read User EEPROM Memory Location
    CmdRoboclawData(0xFD, 3, 4),    // Write User EEPROM Memory Location
    CmdRoboclawData(0xFE, 21, 1),   //
    CmdRoboclawData(0xFF, 0, 0)     //
};

#endif
