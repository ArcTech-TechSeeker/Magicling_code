#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\JsonArrayConst\\size.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("JsonArrayConst::size()") {
  JsonDocument doc;
  JsonArrayConst array = doc.to<JsonArray>();

  SECTION("returns 0 if unbound") {
    JsonArrayConst unbound;
    REQUIRE(0U == unbound.size());
  }

  SECTION("returns 0 is empty") {
    REQUIRE(0U == array.size());
  }

  SECTION("return number of elements") {
    doc.add("hello");
    doc.add("world");

    REQUIRE(2U == array.size());
  }
}
