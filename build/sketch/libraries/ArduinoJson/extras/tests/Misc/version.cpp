#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Misc\\version.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson/version.hpp>
#include <catch.hpp>
#include <sstream>

using Catch::Matchers::StartsWith;

TEST_CASE("ARDUINOJSON_VERSION") {
  std::stringstream version;

  version << ARDUINOJSON_VERSION_MAJOR << "." << ARDUINOJSON_VERSION_MINOR
          << "." << ARDUINOJSON_VERSION_REVISION;

  REQUIRE_THAT(ARDUINOJSON_VERSION, StartsWith(version.str()));
}
