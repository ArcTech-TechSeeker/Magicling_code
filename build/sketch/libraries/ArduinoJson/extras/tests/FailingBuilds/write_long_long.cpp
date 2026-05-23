#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\FailingBuilds\\write_long_long.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#define ARDUINOJSON_USE_LONG_LONG 0
#include <ArduinoJson.h>

#if defined(__SIZEOF_LONG__) && __SIZEOF_LONG__ >= 8
#  error This test requires sizeof(long) < 8
#endif

int main() {
  JsonDocument doc;
  doc["dummy"] = static_cast<long long>(42);
}
