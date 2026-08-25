#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Deprecated\\shallowCopy.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("shallowCopy()") {
  JsonDocument doc1, doc2;
  doc1["b"] = "c";
  doc2["a"].shallowCopy(doc1);

  REQUIRE(doc2.as<std::string>() == "{\"a\":{\"b\":\"c\"}}");
}
