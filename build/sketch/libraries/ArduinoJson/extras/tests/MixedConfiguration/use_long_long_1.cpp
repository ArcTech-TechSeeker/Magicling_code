#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\MixedConfiguration\\use_long_long_1.cpp"
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>

#include <catch.hpp>

TEST_CASE("ARDUINOJSON_USE_LONG_LONG == 1") {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();

  root["A"] = 123456789123456789;
  root["B"] = 987654321987654321;

  std::string json;
  serializeJson(doc, json);

  REQUIRE(json == "{\"A\":123456789123456789,\"B\":987654321987654321}");
}
