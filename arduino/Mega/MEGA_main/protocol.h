#pragma once
#include <Arduino.h>

// Petit helper: envoie une ligne JSON terminée par '\n'
inline void send_json(const String& s) { Serial.println(s); }

// Encode un événement simple: {"src":"mega","event":"..." }
inline void send_event(const __FlashStringHelper* evt) {
  Serial.print(F("{\"src\":\"mega\",\"event\":\""));
  Serial.print(evt);
  Serial.println(F("\"}"));
}
