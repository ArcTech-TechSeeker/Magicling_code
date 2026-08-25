#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Cpp20\\smoke_test.cpp"
#include <ArduinoJson.h>

#include <catch.hpp>
#include <string>

TEST_CASE("C++20 smoke test") {
  JsonDocument doc;

  deserializeJson(doc, "{\"hello\":\"world\"}");
  REQUIRE(doc["hello"] == "world");

  std::string json;
  serializeJson(doc, json);
  REQUIRE(json == "{\"hello\":\"world\"}");
}
