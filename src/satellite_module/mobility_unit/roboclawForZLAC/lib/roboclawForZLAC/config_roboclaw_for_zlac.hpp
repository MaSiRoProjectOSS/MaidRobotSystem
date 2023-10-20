/**
 * @file config_roboclaw_for_zlac.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-13
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef CONFIG_ROBOCLAW_FOR_ZLAC_HPP
#define CONFIG_ROBOCLAW_FOR_ZLAC_HPP

#ifndef SETTING_HEARTBEAT_INTERVAL_MS
#define SETTING_HEARTBEAT_INTERVAL_MS (150)
#endif

/////////////////////////////////////////////////////////

#ifndef ROBOCLAW_CURRENT_MODIFICATION
#define ROBOCLAW_CURRENT_MODIFICATION (15000)
#endif

#ifndef ROBOCLAW_FEED_BACK_TIMES
//#define ROBOCLAW_FEED_BACK_TIMES (4294967295 / 4096)
#define ROBOCLAW_FEED_BACK_TIMES (1)
#endif

/////////////////////////////////////////////////////////

#ifndef PASS_RANGE_ID
#define PASS_RANGE_ID (1)
#endif
#ifndef DEBUG_ZLAC706_SERIAL
#define DEBUG_ZLAC706_SERIAL (0)
#endif

#define DEBUG_ZLAC706_SERIAL_MODE_SPEED    (1)
#define DEBUG_ZLAC706_SERIAL_MODE_POSITION (2)
#define DEBUG_ZLAC706_SERIAL_MODE_TORQUE   (3)

#ifndef DEBUG_ZLAC706_SERIAL_MODE
#define DEBUG_ZLAC706_SERIAL_MODE DEBUG_ZLAC706_SERIAL_MODE_SPEED
#endif

#ifndef ROBOCLAW_SETTING_SET_GAIN
#define ROBOCLAW_SETTING_SET_GAIN (1)
#endif

#endif
