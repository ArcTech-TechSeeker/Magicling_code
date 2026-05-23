#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\src\\ArduinoJson\\Polyfills\\math.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <ArduinoJson/Namespace.hpp>

ARDUINOJSON_BEGIN_PRIVATE_NAMESPACE

// Some libraries #define isnan() and isinf() so we need to check before
// using this name

#ifndef isnan
template <typename T>
bool isnan(T x) {
  return x != x;
}
#endif

#ifndef isinf
template <typename T>
bool isinf(T x) {
  return x != 0.0 && x * 2 == x;
}
#endif
ARDUINOJSON_END_PRIVATE_NAMESPACE
