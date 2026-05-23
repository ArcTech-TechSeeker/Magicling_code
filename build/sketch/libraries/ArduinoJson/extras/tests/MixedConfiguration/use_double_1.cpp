#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\MixedConfiguration\\use_double_1.cpp"
#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>

#include <catch.hpp>

TEST_CASE("ARDUINOJSON_USE_DOUBLE == 1") {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();

  root["pi"] = 3.14;
  root["e"] = 2.72;

  std::string json;
  serializeJson(doc, json);

  REQUIRE(json == "{\"pi\":3.14,\"e\":2.72}");
}
