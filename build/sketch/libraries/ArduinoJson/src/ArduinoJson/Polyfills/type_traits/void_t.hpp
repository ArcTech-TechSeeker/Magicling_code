#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\src\\ArduinoJson\\Polyfills\\type_traits\\void_t.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <ArduinoJson/Namespace.hpp>

ARDUINOJSON_BEGIN_PRIVATE_NAMESPACE

template <typename...>
struct make_void {
  using type = void;
};

template <typename... Args>
using void_t = typename make_void<Args...>::type;
// NOTE: using void_t = void; doesn't work on GCC 4.8

ARDUINOJSON_END_PRIVATE_NAMESPACE
