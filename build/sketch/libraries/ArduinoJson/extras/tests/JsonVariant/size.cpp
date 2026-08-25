#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\JsonVariant\\size.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("JsonVariant::size()") {
  JsonDocument doc;
  JsonVariant variant = doc.to<JsonVariant>();

  SECTION("unbound reference") {
    JsonVariant unbound;

    CHECK(unbound.size() == 0);
  }

  SECTION("int") {
    variant.set(42);

    CHECK(variant.size() == 0);
  }

  SECTION("string") {
    variant.set("hello");

    CHECK(variant.size() == 0);
  }

  SECTION("object") {
    variant["a"] = 1;
    variant["b"] = 2;

    CHECK(variant.size() == 2);
  }
}
