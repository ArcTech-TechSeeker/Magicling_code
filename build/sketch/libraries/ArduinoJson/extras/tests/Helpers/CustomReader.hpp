#line 1 "C:\\Users\\naka6\\Projects\\Magicling_code\\Magicling_code\\libraries\\ArduinoJson\\extras\\tests\\Helpers\\CustomReader.hpp"
// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#pragma once

#include <sstream>

class CustomReader {
  std::stringstream stream_;

 public:
  CustomReader(const char* input) : stream_(input) {}
  CustomReader(const CustomReader&) = delete;

  int read() {
    return stream_.get();
  }

  size_t readBytes(char* buffer, size_t length) {
    stream_.read(buffer, static_cast<std::streamsize>(length));
    return static_cast<size_t>(stream_.gcount());
  }
};
