#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\src\\ArduinoJson\\Polyfills\\type_traits\\remove_cv.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <ArduinoJson/Namespace.hpp>

ARDUINOJSON_BEGIN_PRIVATE_NAMESPACE

template <typename T>
struct remove_cv {
  using type = T;
};
template <typename T>
struct remove_cv<const T> {
  using type = T;
};
template <typename T>
struct remove_cv<volatile T> {
  using type = T;
};
template <typename T>
struct remove_cv<const volatile T> {
  using type = T;
};

template <typename T>
using remove_cv_t = typename remove_cv<T>::type;

ARDUINOJSON_END_PRIVATE_NAMESPACE
