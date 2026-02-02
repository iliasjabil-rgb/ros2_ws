#pragma once
#include <Arduino.h>

#define UNO_BOARD_ID    "002UC_ARDUINO_UNO"
#define UNO_FW_VERSION  "1.0"

// I2C (centrale inertielle 001CI)
static const uint8_t UNO_I2C_SDA = A4;
static const uint8_t UNO_I2C_SCL = A5;

// Relais 001XR..007XR
static const uint8_t UNO_001XR_PIN = 12;//Ruban_led_blanc
static const uint8_t UNO_002XR_PIN = 13;//Ruban_led_rgb
static const uint8_t UNO_003XR_PIN = 7;//Aspirateur
static const uint8_t UNO_004XR_PIN = 2;//Electro-vanne 1 à tester
static const uint8_t UNO_005XR_PIN = 5;//Electro-vanne 2 à tester
static const uint8_t UNO_006XR_PIN = 6;//Electro-vanne 3

// Servos 001..004SV
static const uint8_t UNO_001SV_PIN = 3;// rotation caméra Z
static const uint8_t UNO_002SV_PIN = 4;//rotation caméra Y
static const int8_t  UNO_003SV_PIN = 11;// rotation pince
static const int8_t  UNO_004SV_PIN = 9; // serrage pince

// LED ruban (NeoPixel) — choisis une broche dédiée si utilisé
static const uint8_t UNO_001LED_PIN   = 8;      // D8
static const uint8_t UNO_001LED_COUNT = 144;      // nb de LEDs de ton ruban


