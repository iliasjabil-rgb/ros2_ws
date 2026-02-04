/*
 * MEGA_MAIN_FINAL_V3.ino
 * Gestion Moteurs Pas-à-pas + Sécurité Fin de course + Puissance
 */

#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "pins_mega.h"

// ================= CONFIGURATION =================
const bool HAS_INA260 = true; // Mettre à false si pas de capteur de courant

// Sécurité : Sens du mouvement vers le capteur (-1 ou 1)
const int SENS_VERS_CAPTEUR_01 = -1; 

// ================= GLOBALES =================
AccelStepper ST1(AccelStepper::DRIVER, MEGA_001MO_STEP, MEGA_001MO_DIR);
AccelStepper ST2(AccelStepper::DRIVER, MEGA_002MO_STEP, MEGA_002MO_DIR);
AccelStepper ST3(AccelStepper::DRIVER, MEGA_003MO_STEP, MEGA_003MO_DIR);
AccelStepper ST4(AccelStepper::DRIVER, MEGA_004MO_STEP, MEGA_004MO_DIR);

Adafruit_INA260 ina260;
bool g_ina_found = false;

JsonDocument docRx; // Buffer réception
JsonDocument docTx; // Buffer émission

unsigned long lastFastTelem = 0; // Pos + Limits
unsigned long lastSlowTelem = 0; // Power

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // Init Pins
  pinMode(MEGA_001MO_EN, OUTPUT); pinMode(MEGA_002MO_EN, OUTPUT);
  pinMode(MEGA_003MO_EN, OUTPUT); pinMode(MEGA_004MO_EN, OUTPUT);
  pinMode(MEGA_001MR_PIN, INPUT_PULLUP); // LOW = Activé

  // Init Steppers
  ST1.setPinsInverted(false, false, true); // Enable inversé
  ST2.setPinsInverted(false, false, true);
  ST3.setPinsInverted(false, false, true);
  ST4.setPinsInverted(false, false, true);

  ST1.setMaxSpeed(1500); ST1.setAcceleration(1000);
  ST2.setMaxSpeed(1500); ST2.setAcceleration(1000);
  ST3.setMaxSpeed(1500); ST3.setAcceleration(1000);
  ST4.setMaxSpeed(1500); ST4.setAcceleration(1000);

  ST1.enableOutputs(); ST2.enableOutputs(); 
  ST3.enableOutputs(); ST4.enableOutputs();

  // Init INA260
  if (HAS_INA260 && ina260.begin()) {
    g_ina_found = true;
  }

  Serial.println(F("{\"event\":\"boot\",\"status\":\"ready\"}"));
}

// ================= LOOP =================
void loop() {
  // 1. SÉCURITÉ HARDWARE (Axe 1)
  bool limit1_hit = (digitalRead(MEGA_001MR_PIN) == LOW);
  
  // Si capteur touché ET on essaye d'aller encore plus loin -> STOP NET
  if (limit1_hit && (ST1.distanceToGo() * SENS_VERS_CAPTEUR_01 > 0)) {
     ST1.moveTo(ST1.currentPosition()); // Annule le reste du mouvement
     ST1.stop();
  } else {
     ST1.run();
  }

  // Autres axes (sans limites câblées pour l'instant)
  ST2.run(); ST3.run(); ST4.run();

  // 2. LECTURE SÉRIE
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) handleCmd(input);
  }

  // 3. TÉLÉMÉTRIE RAPIDE (10Hz) : Positions + Limites
  if (millis() - lastFastTelem > 100) {
    lastFastTelem = millis();
    sendPositions();
    sendLimits();
  }

  // 4. TÉLÉMÉTRIE LENTE (2Hz) : Puissance
  if (HAS_INA260 && g_ina_found && (millis() - lastSlowTelem > 500)) {
    lastSlowTelem = millis();
    sendPower();
  }
}

// ================= FONCTIONS =================

void handleCmd(String line) {
  docRx.clear();
  DeserializationError error = deserializeJson(docRx, line);
  if (error) return;

  const char* cmd = docRx["cmd"];

  if (strcmp(cmd, "move") == 0) {
    int axis = docRx["axis"];
    long steps = docRx["steps"];
    float speed = docRx["speed"];
    
    AccelStepper* s = (axis==1)?&ST1 : (axis==2)?&ST2 : (axis==3)?&ST3 : &ST4;
    if (speed > 0) s->setMaxSpeed(speed);
    s->move(steps); // Relatif
  }
  else if (strcmp(cmd, "stop") == 0) {
    // Stop tout et vide les buffers
    ST1.stop(); ST1.moveTo(ST1.currentPosition());
    ST2.stop(); ST2.moveTo(ST2.currentPosition());
    ST3.stop(); ST3.moveTo(ST3.currentPosition());
    ST4.stop(); ST4.moveTo(ST4.currentPosition());
  }
  else if (strcmp(cmd, "home") == 0) {
    // Reset logiciel du zéro (pas de mouvement physique ici)
    int axis = docRx["axis"];
    if(axis == 0) { // Si 0 ou absent -> Tous
        ST1.setCurrentPosition(0); ST2.setCurrentPosition(0);
        ST3.setCurrentPosition(0); ST4.setCurrentPosition(0);
    } else {
        if(axis==1) ST1.setCurrentPosition(0);
        if(axis==2) ST2.setCurrentPosition(0);
        if(axis==3) ST3.setCurrentPosition(0);
        if(axis==4) ST4.setCurrentPosition(0);
    }
  }
}

void sendPositions() {
  docTx.clear();
  docTx["src"] = "mega";
  JsonArray pos = docTx["pos"].to<JsonArray>();
  pos.add(ST1.currentPosition());
  pos.add(ST2.currentPosition());
  pos.add(ST3.currentPosition());
  pos.add(ST4.currentPosition());
  serializeJson(docTx, Serial); Serial.println();
}

void sendLimits() {
  docTx.clear();
  docTx["src"] = "mega";
  JsonArray mr = docTx["mr"].to<JsonArray>();
  // 1 = Capteur activé, 0 = libre (Inversion de logic si INPUT_PULLUP)
  mr.add(digitalRead(MEGA_001MR_PIN) == LOW ? 1 : 0);
  mr.add(0); // Placeholder Axe 2
  mr.add(0); // Placeholder Axe 3
  mr.add(0); // Placeholder Axe 4
  serializeJson(docTx, Serial); Serial.println();
}

void sendPower() {
  docTx.clear();
  docTx["src"] = "mega";
  JsonArray pwr = docTx["pwr"].to<JsonArray>();
  pwr.add(ina260.readBusVoltage()); // mV
  pwr.add(ina260.readCurrent());    // mA
  pwr.add(ina260.readPower());      // mW
  serializeJson(docTx, Serial); Serial.println();
}