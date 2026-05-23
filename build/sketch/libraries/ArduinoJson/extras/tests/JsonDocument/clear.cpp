#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\JsonDocument\\clear.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

#include <stdlib.h>  // malloc, free
#include <string>

#include "Allocators.hpp"
#include "Literals.hpp"

TEST_CASE("JsonDocument::clear()") {
  SpyingAllocator spy;
  JsonDocument doc(&spy);

  SECTION("null") {
    doc.clear();

    REQUIRE(doc.isNull());
    REQUIRE(spy.log() == AllocatorLog{});
  }

  SECTION("releases resources") {
    doc["hello"_s] = "world"_s;
    spy.clearLog();

    doc.clear();

    REQUIRE(doc.isNull());
    REQUIRE(spy.log() == AllocatorLog{
                             Deallocate(sizeofPool()),
                             Deallocate(sizeofString("hello")),
                             Deallocate(sizeofString("world")),
                         });
  }

  SECTION("clear free list") {  // issue #2034
    JsonObject obj = doc.to<JsonObject>();
    obj["a"] = 1;
    obj.clear();  // puts the slot in the free list

    doc.clear();

    doc["b"] = 2;  // will it pick from the free list?
  }
}
