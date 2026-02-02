#pragma once
#include <Arduino.h>
inline void send_json(const String& s) { Serial.println(s); }

// pour F("...")
inline void send_json(const __FlashStringHelper* s) {
  Serial.println(s);
}