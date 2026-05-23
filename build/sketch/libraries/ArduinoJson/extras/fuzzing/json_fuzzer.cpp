#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\fuzzing\\json_fuzzer.cpp"
#include <ArduinoJson.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, data, size);
  if (!error) {
    std::string json;
    serializeJson(doc, json);
  }
  return 0;
}
