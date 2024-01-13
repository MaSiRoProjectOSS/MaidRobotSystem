/**
 * @file arduino_characters.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_CHARACTERS_HPP
#define ARDUINO_FUNC_ARDUINO_CHARACTERS_HPP

bool isAlpha(char this_char);
bool isAlphaNumeric(char this_char);
bool isAscii(char this_char);
bool isControl(char this_char);
bool isDigit(char this_char);
bool isGraph(char this_char);
bool isHexadecimalDigit(char this_char);
bool isLowerCase(char this_char);
bool isPrintable(char this_char);
bool isPunct(char this_char);
bool isSpace(char this_char);
bool isUpperCase(char this_char);
bool isWhitespace(char this_char);

#endif
