#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Misc\\custom_string.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <string>

struct custom_char_traits : std::char_traits<char> {};

using custom_string = std::basic_string<char, custom_char_traits>;
