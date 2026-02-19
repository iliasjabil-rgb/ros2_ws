#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 * FICHIER : protocol.h
 * ROLE    : Fonctions utilitaires pour structurer la communication JSON 
 * entre la carte Mega et le Raspberry Pi (ROS 2).
 * ============================================================================
 */

/**
 * @brief Envoie une ligne de texte (idéalement du JSON) sur le port Série.
 * * @param s La chaîne de caractères (String) contenant le message.
 * @note  La fonction ajoute automatiquement le '\n' (retour à la ligne) 
 * à la fin, ce qui est indispensable pour que le script Python 
 * côté ROS 2 détecte la fin du message.
 */
inline void send_json(const String& s) { 
  Serial.println(s); 
}

/**
 * @brief Formate et envoie un événement système simple au format JSON.
 * * @param evt Le nom de l'événement stocké en mémoire Flash (via la macro F()).
 * L'utilisation de F() permet d'économiser la précieuse RAM de l'Arduino.
 * * @note  Le format final généré sera : {"src":"mega","event":"<evt>"}
 * @example send_event(F("emergency_stop"));
 */
inline void send_event(const __FlashStringHelper* evt) {
  Serial.print(F("{\"src\":\"mega\",\"event\":\""));
  Serial.print(evt);
  Serial.println(F("\"}"));
}