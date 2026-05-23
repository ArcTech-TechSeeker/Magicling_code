#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\MixedConfiguration\\decode_unicode_1.cpp"
#define ARDUINOJSON_DECODE_UNICODE 1
#include <ArduinoJson.h>

#include <catch.hpp>

TEST_CASE("ARDUINOJSON_DECODE_UNICODE == 1") {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, "\"\\uD834\\uDD1E\"");

  REQUIRE(err == DeserializationError::Ok);
}
