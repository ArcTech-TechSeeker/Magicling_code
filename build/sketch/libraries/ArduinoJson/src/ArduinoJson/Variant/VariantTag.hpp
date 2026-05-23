#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\src\\ArduinoJson\\Variant\\VariantTag.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <ArduinoJson/Namespace.hpp>

ARDUINOJSON_BEGIN_PRIVATE_NAMESPACE

struct VariantTag {};

template <typename T>
struct IsVariant : is_base_of<VariantTag, T> {};

ARDUINOJSON_END_PRIVATE_NAMESPACE
