#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Misc\\issue2166.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

struct CCLASS {
  static const char mszKey[];
};

TEST_CASE("Issue #2166") {
  JsonDocument doc;
  doc[CCLASS::mszKey] = 12;
  REQUIRE(doc.as<std::string>() == "{\"test3\":12}");

  JsonObject obj = doc.to<JsonObject>();
  obj[CCLASS::mszKey] = 12;
  REQUIRE(doc.as<std::string>() == "{\"test3\":12}");
}

const char CCLASS::mszKey[] = "test3";
