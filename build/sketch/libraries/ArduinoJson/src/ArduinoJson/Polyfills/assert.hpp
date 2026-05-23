#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\src\\ArduinoJson\\Polyfills\\assert.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <ArduinoJson/Configuration.hpp>

#if ARDUINOJSON_DEBUG
#  include <assert.h>
#  define ARDUINOJSON_ASSERT(X) assert(X)
#else
#  define ARDUINOJSON_ASSERT(X) ((void)0)
#endif
