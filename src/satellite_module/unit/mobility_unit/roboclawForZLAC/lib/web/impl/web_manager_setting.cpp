/**
 * @file web_manager_setting.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.1
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#include "web_manager_setting.hpp"

#include "../web_setting.hpp"

#include <Arduino.h>

#if SETTING_STORAGE_SPI_FS
#include <SPIFFS.h>
#endif

namespace MaSiRoProject
{
namespace Web
{

WebManagerSetting::WebManagerSetting()
{
    this->_hostname = "";
    (void)this->_default_information();
    this->_open_fs = false;
}

bool WebManagerSetting::_setup()
{
    (void)this->_default_information();
    this->_open_fs = false;

    bool result = false;
#if SETTING_STORAGE_SPI_FS
    if (0 <= this->_error_count_spi) {
        // SPI FFS doing format if happened error
        if (true == SPIFFS.begin(false)) {
#if SETTING_STORAGE_OVERRIDE
            this->_save_information(this->_ssid, this->_pass, this->_mode_ap, this->_auto_default_setting);
#endif
            if (true == SPIFFS.exists(SETTING_WIFI_SETTING_FILE)) {
                // load setting information from file
                result = this->_load_information();
            } else {
                // save setting information because File is not exists
                result = this->_save_information(this->_ssid, this->_pass, this->_mode_ap, this->_auto_default_setting);
            }
            SPIFFS.end();
        } else {
            this->_error_count_spi--;
        }
        this->_open_fs = result;
    } else {
        // override setting information
        this->_open_fs = false;
        result         = this->save_information(this->_ssid, this->_pass, this->_mode_ap, this->_auto_default_setting);
    }
#else
    // override setting information
    result = this->save_information(this->_ssid, this->_pass, this->_mode_ap, this->_auto_default_setting);
#endif
    return result;
}

bool WebManagerSetting::save_information(std::string ssid, std::string pass, bool ap_mode, bool auto_default)
{
    bool result = false;
#if SETTING_STORAGE_SPI_FS
    bool force_write = false;
    if (true == this->_open_fs) {
        if (true == SPIFFS.begin()) {
            if (true == SPIFFS.exists(SETTING_WIFI_SETTING_FILE)) {
                this->_load_information();
            } else {
                force_write = true;
            }
            if ((true == force_write)                                                            //
                || (this->_ssid != ssid) || (this->_pass != pass) || (this->_mode_ap != ap_mode) //
                || (this->_auto_default_setting != auto_default)) {
                force_write = true;

                result = _save_information(ssid, pass, ap_mode, auto_default);
            }
            SPIFFS.end();
            if (true == force_write) {
                ssid         = this->_pass;
                pass         = this->_pass;
                ap_mode      = this->_mode_ap;
                auto_default = this->_auto_default_setting;
            }
        }
    } else {
        if (0 > this->_error_count_spi) {
            result = true;
        }
    }
#else
    result = true;
#endif
    (void)this->_set_information(ssid, pass, ap_mode, auto_default);
    return result;
}
bool WebManagerSetting::load_information()
{
    bool result = false;
#if SETTING_STORAGE_SPI_FS
    if (true == this->_open_fs) {
        if (SPIFFS.begin()) {
            if (true == SPIFFS.exists(SETTING_WIFI_SETTING_FILE)) {
                result = this->_load_information();
            }
            SPIFFS.end();
        }
    } else {
        if (0 > this->_error_count_spi) {
            this->_default_information();
            result = true;
        }
    }
#else
    this->_default_information();
    result = true;
#endif
    return result;
}

bool WebManagerSetting::_default_information()
{
    bool result = false;
    (void)this->_set_information(SETTING_WIFI_SSID_DEFAULT, SETTING_WIFI_PASS_DEFAULT, SETTING_WIFI_MODE_AP, SETTING_WIFI_MODE_AUTO_TRANSITIONS);
    return result;
}

void WebManagerSetting::set_information(std::string ssid, std::string pass, bool ap_mode, bool auto_default)
{
    this->_set_information(ssid, pass, ap_mode, auto_default);
}
////////////////////////////////////////////////////
// private function
////////////////////////////////////////////////////
void WebManagerSetting::_set_information(std::string ssid, std::string pass, bool ap_mode, bool auto_default)
{
    this->_ssid                 = ssid;
    this->_pass                 = pass;
    this->_mode_ap              = ap_mode;
    this->_auto_default_setting = auto_default;
}

bool WebManagerSetting::_load_information()
{
    bool result = false;
#if SETTING_STORAGE_SPI_FS
    int totalBytes = 0;
    int type       = 0;
    if (true == SPIFFS.exists(SETTING_WIFI_SETTING_FILE)) {
        File dataFile = SPIFFS.open(SETTING_WIFI_SETTING_FILE, FILE_READ);
        if (!dataFile) {
            result = false;
        } else {
            result     = true;
            totalBytes = dataFile.size();
            while (dataFile.available()) {
                String word = dataFile.readStringUntil('\n');
                switch (type) {
                    case 0:
                        if (true == word.equals("A")) {
                            this->_mode_ap = true;
                        } else {
                            this->_mode_ap = false;
                        }
                        break;
                    case 1:
                        if (0 < word.length()) {
                            this->_ssid = word.c_str();
                        }
                        break;
                    case 2:
                        if (0 < word.length()) {
                            this->_pass = word.c_str();
                        }
                        break;
                    case 3:
                        if (true == word.equals("Y")) {
                            this->_auto_default_setting = true;
                        } else {
                            this->_auto_default_setting = false;
                        }
                        break;
                    case 4:
                        if (0 < word.length()) {
                            this->_hostname = word.c_str();
                        } else {
                            this->_hostname = "";
                        }
                        break;

                    default:
                        break;
                }
                type++;
                if (4 < type) {
                    break;
                }
            }
            dataFile.close();
        }
    }
#else
    this->_default_information();
    result = true;
#endif
    return result;
}
bool WebManagerSetting::_save_information(std::string ssid, std::string pass, bool ap_mode, bool auto_default)
{
    bool result = false;
#if SETTING_STORAGE_SPI_FS
    File dataFile = SPIFFS.open(SETTING_WIFI_SETTING_FILE, FILE_WRITE);
    dataFile.println((true == ap_mode) ? "A" : "S");
    dataFile.println(ssid.c_str());
    dataFile.println(pass.c_str());
    dataFile.println((true == auto_default) ? "Y" : "N");
    dataFile.println(this->_hostname.c_str());
    dataFile.close();
    result = true;
#else
    result = true;
#endif
    return result;
}

void WebManagerSetting::set_hostname(std::string hostname)
{
    this->_hostname = hostname;
}

} // namespace Web
} // namespace MaSiRoProject
