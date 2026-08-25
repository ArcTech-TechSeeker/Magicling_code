#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\FailingBuilds\\assign_char.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>

// See issue #1498

int main() {
  JsonDocument doc;
  doc["dummy"] = 'A';
}
