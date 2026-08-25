#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Deprecated\\add.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("JsonArray::add()") {
  JsonDocument doc;
  JsonArray array = doc.to<JsonArray>();
  array.add().set(42);
  REQUIRE(doc.as<std::string>() == "[42]");
}

TEST_CASE("JsonDocument::add()") {
  JsonDocument doc;
  doc.add().set(42);
  REQUIRE(doc.as<std::string>() == "[42]");
}

TEST_CASE("ElementProxy::add()") {
  JsonDocument doc;
  doc[0].add().set(42);
  REQUIRE(doc.as<std::string>() == "[[42]]");
}

TEST_CASE("MemberProxy::add()") {
  JsonDocument doc;
  doc["x"].add().set(42);
  REQUIRE(doc.as<std::string>() == "{\"x\":[42]}");
}

TEST_CASE("JsonVariant::add()") {
  JsonDocument doc;
  JsonVariant v = doc.add();
  v.add().set(42);
  REQUIRE(doc.as<std::string>() == "[[42]]");
}
