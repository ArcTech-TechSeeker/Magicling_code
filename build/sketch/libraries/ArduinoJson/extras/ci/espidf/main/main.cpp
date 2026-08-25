#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\ci\\espidf\\main\\main.cpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>

extern "C" void app_main() {
  char buffer[256];
  JsonDocument doc;

  doc["hello"] = "world";
  serializeJson(doc, buffer);
  deserializeJson(doc, buffer);
  serializeMsgPack(doc, buffer);
  deserializeMsgPack(doc, buffer);
}
