#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\src\\ArduinoJson\\Polyfills\\type_traits\\is_class.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include "declval.hpp"

ARDUINOJSON_BEGIN_PRIVATE_NAMESPACE

template <typename T>
struct is_class {
 protected:  // <- to avoid GCC's "all member functions in class are private"
  template <typename U>
  static int probe(void (U::*)(void));
  template <typename>
  static char probe(...);

 public:
  static const bool value = sizeof(probe<T>(0)) == sizeof(int);
};

ARDUINOJSON_END_PRIVATE_NAMESPACE
