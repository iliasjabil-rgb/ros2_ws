#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 * FICHIER : pins_mega.h
 * ROLE    : Définition du mapping matériel (broches) pour l'Arduino Mega.
 * Centralise toutes les connexions physiques du robot.
 * ============================================================================
 */

/* --- IDENTIFICATION DU NŒUD --- */
#define MEGA_BOARD_ID   "001UC_ARDUINO_MEGA"
#define MEGA_FW_VERSION "1.0"

/*** ======== MOTEURS PAS-A-PAS (DRIVERS 002DM) =========
 *  001MO : Colonne
 *  002MO : Stockage
 *  003MO : Déplacement tige
 *  004MO : Tige rétractable

/**
 * ============================================================================
 * MOTEURS PAS-À-PAS (DRIVERS 002DM)
 * ============================================================================
 * Chaque driver moteur utilise 3 broches de contrôle :
 * - STEP : Impulsion pour avancer d'un pas
 * - DIR  : Sens de rotation
 * - EN   : Activation / Désactivation de la puissance (Enable)
 */

// Moteur 1 (001MO) : Colonne Z (Élévation principale)
static const uint8_t MEGA_001MO_STEP = 22;
static const uint8_t MEGA_001MO_DIR  = 23;
static const uint8_t MEGA_001MO_EN   = 24;

// Moteur 2 (002MO) : Stockage (Gestion du magasin)
static const uint8_t MEGA_002MO_STEP = 25;
static const uint8_t MEGA_002MO_DIR  = 26;
static const uint8_t MEGA_002MO_EN   = 27;

// Moteur 3 (003MO) : Déplacement tige axe Y (Avance/Recul horizontal)
static const uint8_t MEGA_003MO_STEP = 28;
static const uint8_t MEGA_003MO_DIR  = 29;
static const uint8_t MEGA_003MO_EN   = 30;

// Moteur 4 (004MO) : Tige rétractable (Vérin à vis)
static const uint8_t MEGA_004MO_STEP = 31;
static const uint8_t MEGA_004MO_DIR  = 32;
static const uint8_t MEGA_004MO_EN   = 33;


/**
 * ============================================================================
 * MICRORUPTEURS / FINS DE COURSE (001MR - 009MR)
 * ============================================================================
 * Doivent être configurés en INPUT_PULLUP dans le setup().
 * État logique : 
 * - LOW  (0) = Switch pressé (connecté physiquement à la masse GND).
 * - HIGH (1) = Switch relâché (tiré à 5V par la résistance interne).
 */
static const uint8_t MEGA_001MR_PIN  = 34;
static const uint8_t MEGA_002MR_PIN  = 35;
static const uint8_t MEGA_003MR_PIN  = 36;
static const uint8_t MEGA_004MR_PIN  = 37;
static const uint8_t MEGA_005MR_PIN  = 38;
static const uint8_t MEGA_006MR_PIN  = 39;
static const uint8_t MEGA_007MR_PIN  = 40;
static const uint8_t MEGA_008MR_PIN  = 41;
static const uint8_t MEGA_009MR_PIN  = 42;


/**
 * ============================================================================
 * ALARMES DRIVERS (ALM MOTEURS)
 * ============================================================================
 * Signal de défaut (Fault) remonté par chaque driver moteur (bloqué, surchauffe).
 * Doivent être configurés en INPUT_PULLUP.
 * État logique : Actif à LOW si le driver se met en sécurité.
 */
static const uint8_t MEGA_001MO_ALM = 8;  // Alarme Colonne Z
// Le moteur 2 n'a pas de pin ALM déclaré
static const uint8_t MEGA_003MO_ALM = 9;  // Alarme Axe Y
static const uint8_t MEGA_004MO_ALM = 10; // Alarme Vérin à vis
