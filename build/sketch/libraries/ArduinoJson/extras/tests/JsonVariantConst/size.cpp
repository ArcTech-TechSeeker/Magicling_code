#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\JsonVariantConst\\size.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("JsonVariantConst::size()") {
  JsonDocument doc;
  JsonVariantConst variant = doc.to<JsonVariant>();

  SECTION("unbound reference") {
    JsonVariantConst unbound;

    CHECK(unbound.size() == 0);
  }

  SECTION("int") {
    doc.set(42);

    CHECK(variant.size() == 0);
  }

  SECTION("string") {
    doc.set("hello");

    CHECK(variant.size() == 0);
  }

  SECTION("object") {
    doc["a"] = 1;
    doc["b"] = 2;

    CHECK(variant.size() == 2);
  }
}
