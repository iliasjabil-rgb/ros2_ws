#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 * FICHIER : protocol.h (UNO)
 * ROLE    : Fonctions utilitaires pour structurer la communication JSON 
 * entre la carte Uno et le Raspberry Pi (ROS 2).
 * ============================================================================
 */

/**
 * @brief Envoie une chaîne de caractères dynamique (String) au format JSON.
 * @param s La chaîne de caractères contenant le message.
 * @note  La fonction ajoute automatiquement le '\n' (retour à la ligne) 
 * indispensable pour que le driver Python (ROS 2) lise la ligne complète.
 */
inline void send_json(const String& s) { 
  Serial.println(s); 
}

/**
 * @brief Envoie une chaîne de caractères stockée en mémoire Flash.
 * @param s Pointeur vers la chaîne générée par la macro F("...").
 * @note  Surcharge essentielle pour la Uno (qui possède très peu de RAM).
 * Permet d'envoyer du JSON statique sans consommer de mémoire dynamique.
 * @example send_json(F("{\"src\":\"uno\",\"event\":\"boot\"}"));
 */
inline void send_json(const __FlashStringHelper* s) {
  Serial.println(s);
}