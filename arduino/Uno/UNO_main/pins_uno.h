#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 * FICHIER : pins_uno.h
 * ROLE    : Définition du mapping matériel (broches) pour l'Arduino Uno.
 * Centralise toutes les connexions pour les actionneurs secondaires, relais, 
 * servos et capteurs I2C.
 * ============================================================================
 */

/* --- IDENTIFICATION DU NŒUD --- */
#define UNO_BOARD_ID    "002UC_ARDUINO_UNO"
#define UNO_FW_VERSION  "1.0"


/**
 * ============================================================================
 * BUS I2C (CENTRALE INERTIELLE 001CI)
 * ============================================================================
 * SDA = Ligne de données (A4 sur Uno)
 * SCL = Ligne d'horloge (A5 sur Uno)
 */
static const uint8_t UNO_I2C_SDA = A4;
static const uint8_t UNO_I2C_SCL = A5;


/**
 * ============================================================================
 * RELAIS / TRANSISTORS (001XR à 006XR)
 * ============================================================================
 * Contrôle des équipements de puissance (lumières, pompe, électrovannes).
 * Généralement actifs à l'état HIGH (1), selon le module relais utilisé.
 */
static const uint8_t UNO_001XR_PIN = 12; // Ruban LED Blanc
static const uint8_t UNO_002XR_PIN = 13; // Ruban LED RGB (Général)
static const uint8_t UNO_003XR_PIN = 7;  // Aspirateur / Pompe
static const uint8_t UNO_004XR_PIN = 2;  // Électrovanne 1 (À tester)
static const uint8_t UNO_005XR_PIN = 5;  // Électrovanne 2 (À tester)
static const uint8_t UNO_006XR_PIN = 6;  // Électrovanne 3


/**
 * ============================================================================
 * SERVOMOTEURS (001SV à 004SV)
 * ============================================================================
 * Contrôle PWM via la librairie Servo.h.
 */
static const uint8_t UNO_001SV_PIN = 3;  // Rotation Caméra (Axe Z / Pan)
static const uint8_t UNO_002SV_PIN = 4;  // Rotation Caméra (Axe Y / Tilt)
static const uint8_t UNO_003SV_PIN = 11; // Rotation Pince (Poignet)
static const uint8_t UNO_004SV_PIN = 9;  // Serrage Pince (Préhension)


/**
 * ============================================================================
 * CAPTEURS ET ENTRÉES
 * ============================================================================
 */
static const uint8_t UNO_001FC_PIN = A0; // Fin de course (Serrage Pince)


/**
 * ============================================================================
 * RUBAN LED ADRESSABLE (NEOPIXEL / WS2812B)
 * ============================================================================
 * Contrôle d'un ruban LED intelligent (chaque LED est indépendante).
 */
static const uint8_t UNO_001LED_PIN   = 8;   // Broche Data du ruban
static const uint16_t UNO_001LED_COUNT = 115; // Nombre total de LEDs sur le ruban
