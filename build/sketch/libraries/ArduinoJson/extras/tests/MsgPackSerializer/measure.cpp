#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\MsgPackSerializer\\measure.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("measureMsgPack()") {
  JsonDocument doc;
  JsonObject object = doc.to<JsonObject>();
  object["hello"] = "world";

  REQUIRE(measureMsgPack(doc) == 13);
}
