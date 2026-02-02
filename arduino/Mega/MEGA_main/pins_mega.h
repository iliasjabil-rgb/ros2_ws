#pragma once
#include <Arduino.h>

/*** ======== IDENTIFICATION ========= ***/
#define MEGA_BOARD_ID   "001UC_ARDUINO_MEGA"
#define MEGA_FW_VERSION "1.0"

/*** ======== MOTEURS PAS-A-PAS (DRIVERS 002DM) =========
 *  001MO : Colonne
 *  002MO : Stockage
 *  003MO : Déplacement tige
 *  004MO : Tige rétractable
 */
static const uint8_t MEGA_001MO_STEP = 22;
static const uint8_t MEGA_001MO_DIR  = 23;
static const uint8_t MEGA_001MO_EN   = 24;

static const uint8_t MEGA_002MO_STEP = 25;
static const uint8_t MEGA_002MO_DIR  = 26;
static const uint8_t MEGA_002MO_EN   = 27;

static const uint8_t MEGA_003MO_STEP = 28;
static const uint8_t MEGA_003MO_DIR  = 29;
static const uint8_t MEGA_003MO_EN   = 30;

static const uint8_t MEGA_004MO_STEP = 31;
static const uint8_t MEGA_004MO_DIR  = 32;
static const uint8_t MEGA_004MO_EN   = 33;

/*** ======== MICRORUPTEURS FIN DE COURSE (001-009MR) ========
 * Entrées avec PULLUP interne (actif à LOW si câblé vers GND)
 */
static const uint8_t MEGA_001MR_PIN = 34;
static const uint8_t MEGA_002MR_PIN = 35;
static const uint8_t MEGA_003MR_PIN = 36;
static const uint8_t MEGA_004MR_PIN = 37;
static const uint8_t MEGA_005MR_PIN = 38;
static const uint8_t MEGA_006MR_PIN = 39;
static const uint8_t MEGA_007MR_PIN = 40;
static const uint8_t MEGA_008MR_PIN = 41;
static const uint8_t MEGA_009MR_PIN = 42;

/*** ======== MOTEUR PRÉHENSEUR / POMPE (si stepper DC ailleurs) ========
 * Si le préhenseur (008MO) est un PAP, mets ses STEP/DIR/EN ici
 * Sinon, laisse non utilisé sur la MEGA.
 */
static const int8_t  MEGA_008MO_STEP = -1;  // -1 = non utilisé
static const int8_t  MEGA_008MO_DIR  = -1;
static const int8_t  MEGA_008MO_EN   = -1;

/*** ======== DIVERS ALIMS / RELAIS éventuels (option) ======== */
static const int8_t  MEGA_SPARE1 = -1;
static const int8_t  MEGA_SPARE2 = -1;

/*** ======== ALARM DRIVERS (ALM global) ======== */
// Entrée avec PULLUP interne (actif à LOW si un driver est en faute)
static const uint8_t MEGA_ALM_PIN = 43;   // à adapter au pin réel utilisé
