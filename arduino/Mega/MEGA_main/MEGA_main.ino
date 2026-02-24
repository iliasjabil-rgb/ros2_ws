/**
 * ============================================================================
 * FICHIER : MEGA_main.ino (V6 - Final & Clean)
 * ROLE    : Gestion 4 Moteurs + 4 Capteurs INA260 + Double Sécurité (Min/Max)
 * + Remontée des alarmes drivers (ALM) vers ROS 2.
 * ============================================================================
 */

#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "pins_mega.h" // Le mapping matériel détaillé

/**
 * ============================================================================
 * CONFIGURATION ET PARAMÈTRES
 * ============================================================================
 */
const bool HAS_INA260 = true; 

// Adresses I2C des 4 capteurs de puissance
#define ADDR_INA1 0x40 // Rien soudé
#define ADDR_INA2 0x41 // A0 soudé
#define ADDR_INA3 0x44 // A1 soudé
#define ADDR_INA4 0x45 // A0 + A1 soudés

// Fréquences d'envoi de la télémétrie (en millisecondes)
const unsigned long TELEMETRY_FAST_MS = 100; // 10 Hz (Positions, Fins de course, Alarmes)
const unsigned long TELEMETRY_SLOW_MS = 500; // 2 Hz  (Consommation INA260)

/**
 * ============================================================================
 * MAPPING DES FINS DE COURSE (PAIRES MIN/MAX)
 * ============================================================================
 */
// Moteur 1 (Colonne Z)
#define PIN_M1_MIN MEGA_001MR_PIN // Pin 34
#define PIN_M1_MAX MEGA_002MR_PIN // Pin 35

// Moteur 2 (Stockage)
#define PIN_M2_MIN MEGA_003MR_PIN // Pin 36
#define PIN_M2_MAX MEGA_004MR_PIN // Pin 37

// Moteur 3 (Tige Y)
#define PIN_M3_MIN MEGA_005MR_PIN // Pin 38
#define PIN_M3_MAX MEGA_006MR_PIN // Pin 39

// Moteur 4 (Vérin rétractable)
#define PIN_M4_MIN MEGA_007MR_PIN // Pin 40
#define PIN_M4_MAX MEGA_008MR_PIN // Pin 41

/**
 * ============================================================================
 * VARIABLES GLOBALES ET OBJETS
 * ============================================================================
 */
// Initialisation des moteurs
AccelStepper ST1(AccelStepper::DRIVER, MEGA_001MO_STEP, MEGA_001MO_DIR);
AccelStepper ST2(AccelStepper::DRIVER, MEGA_002MO_STEP, MEGA_002MO_DIR);
AccelStepper ST3(AccelStepper::DRIVER, MEGA_003MO_STEP, MEGA_003MO_DIR);
AccelStepper ST4(AccelStepper::DRIVER, MEGA_004MO_STEP, MEGA_004MO_DIR);

// Objets Capteurs de puissance
Adafruit_INA260 ina1, ina2, ina3, ina4;

// État de connexion des capteurs
bool ina1_ok = false, ina2_ok = false, ina3_ok = false, ina4_ok = false;

// Buffers de communication
JsonDocument docRx;
JsonDocument docTx;

// Timers
unsigned long lastFastTelem = 0;
unsigned long lastSlowTelem = 0;

// Prototype fonction sécurité
void checkAndRunDual(AccelStepper &stepper, uint8_t pinMin, uint8_t pinMax);

/**
 * ============================================================================
 * SETUP : INITIALISATION
 * ============================================================================
 */
void setup() {
  Serial.begin(115200);

  // --- 1. Init Pins Enable (Sorties) ---
  pinMode(MEGA_001MO_EN, OUTPUT);
  pinMode(MEGA_002MO_EN, OUTPUT);
  pinMode(MEGA_003MO_EN, OUTPUT);
  pinMode(MEGA_004MO_EN, OUTPUT);

  // --- 2. Init Pins Fins de course (Entrées avec Pullup) ---
  pinMode(PIN_M1_MIN, INPUT_PULLUP); pinMode(PIN_M1_MAX, INPUT_PULLUP);
  pinMode(PIN_M2_MIN, INPUT_PULLUP); pinMode(PIN_M2_MAX, INPUT_PULLUP);
  pinMode(PIN_M3_MIN, INPUT_PULLUP); pinMode(PIN_M3_MAX, INPUT_PULLUP);
  pinMode(PIN_M4_MIN, INPUT_PULLUP); pinMode(PIN_M4_MAX, INPUT_PULLUP);

  // --- 3. Init Pins d'Alarme Drivers (ALM) ---
  // (La coupure se fait en hard dans le driver, l'Arduino ne fait que lire)
  pinMode(MEGA_001MO_ALM, INPUT_PULLUP); // Pin 8
  pinMode(MEGA_003MO_ALM, INPUT_PULLUP); // Pin 9
  pinMode(MEGA_004MO_ALM, INPUT_PULLUP); // Pin 10

  // --- 4. Init Steppers ---
  // (Inversion Enable : true car souvent actif à LOW sur drivers type DM542/TB6600)
  ST1.setPinsInverted(false, false, true);
  ST2.setPinsInverted(false, false, true);
  ST3.setPinsInverted(false, false, true);
  ST4.setPinsInverted(false, false, true);

  // Vitesses et Accélérations par défaut
  ST1.setMaxSpeed(1500); ST1.setAcceleration(1000);
  ST2.setMaxSpeed(1500); ST2.setAcceleration(1000);
  ST3.setMaxSpeed(1500); ST3.setAcceleration(1000);
  ST4.setMaxSpeed(1500); ST4.setAcceleration(1000);

  ST1.enableOutputs(); 
  ST2.enableOutputs(); 
  ST3.enableOutputs(); 
  ST4.enableOutputs();

  // --- 5. Init Capteurs INA260 ---
  if (HAS_INA260) {
    if (ina1.begin(ADDR_INA1)) ina1_ok = true;
    if (ina2.begin(ADDR_INA2)) ina2_ok = true;
    if (ina3.begin(ADDR_INA3)) ina3_ok = true;
    if (ina4.begin(ADDR_INA4)) ina4_ok = true;
  }

  Serial.println(F("{\"event\":\"boot\",\"status\":\"ready_v6\"}"));
}

/**
 * ============================================================================
 * LOOP : BOUCLE PRINCIPALE
 * ============================================================================
 */
void loop() {
  
  // --- 1. GESTION DES MOTEURS + DOUBLE SÉCURITÉ ---
  checkAndRunDual(ST1, PIN_M1_MIN, PIN_M1_MAX);
  checkAndRunDual(ST2, PIN_M2_MIN, PIN_M2_MAX);
  checkAndRunDual(ST3, PIN_M3_MIN, PIN_M3_MAX);
  checkAndRunDual(ST4, PIN_M4_MIN, PIN_M4_MAX);

  // --- 2. LECTURE SÉRIE ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) handleCmd(input);
  }

  // --- 3. TÉLÉMÉTRIE RAPIDE (Positions, FdC, Alarmes) ---
  if (millis() - lastFastTelem > TELEMETRY_FAST_MS) {
    lastFastTelem = millis();
    sendPositions();
    sendLimits();
  }

  // --- 4. TÉLÉMÉTRIE LENTE (Puissance) ---
  if (HAS_INA260 && (millis() - lastSlowTelem > TELEMETRY_SLOW_MS)) {
    lastSlowTelem = millis();
    sendPower();
  }
}

/**
 * ============================================================================
 * FONCTIONS : SÉCURITÉ & CONTRÔLE
 * ============================================================================
 */

/**
 * @brief Vérifie les fins de course et autorise ou bloque le mouvement
 */
void checkAndRunDual(AccelStepper &stepper, uint8_t pinMin, uint8_t pinMax) {
  bool hit_min = (digitalRead(pinMin) == LOW);
  bool hit_max = (digitalRead(pinMax) == LOW);
  
  long distance = stepper.distanceToGo(); // Positif = vers Max, Négatif = vers Min

  // SÉCURITÉ MIN : Si capteur touché ET on essaye d'aller encore plus loin en négatif
  if (hit_min && distance < 0) {
      stepper.moveTo(stepper.currentPosition());
      stepper.stop();
  } 
  // SÉCURITÉ MAX : Si capteur touché ET on essaye d'aller encore plus loin en positif
  else if (hit_max && distance > 0) {
      stepper.moveTo(stepper.currentPosition());
      stepper.stop();
  } 
  // Mouvement autorisé
  else {
      stepper.run();
  }
}

/**
 * @brief Parse et exécute les commandes ROS 2
 */
void handleCmd(String line) {
  docRx.clear();
  DeserializationError error = deserializeJson(docRx, line);
  if (error) return;
  
  const char* cmd = docRx["cmd"];

  if (strcmp(cmd, "move") == 0) {
    int axis = docRx["axis"];
    long steps = docRx["steps"];
    float speed = docRx["speed"];
    
    if (axis < 1 || axis > 4) return; // Sécurité anti-crash pointeur
    
    AccelStepper* s = (axis==1)?&ST1 : (axis==2)?&ST2 : (axis==3)?&ST3 : &ST4;
    
    if (speed > 0) s->setMaxSpeed(speed);
    s->move(steps); // Déplacement relatif
  }
  else if (strcmp(cmd, "stop") == 0) {
    ST1.stop(); ST1.moveTo(ST1.currentPosition());
    ST2.stop(); ST2.moveTo(ST2.currentPosition());
    ST3.stop(); ST3.moveTo(ST3.currentPosition());
    ST4.stop(); ST4.moveTo(ST4.currentPosition());
  }
  else if (strcmp(cmd, "home") == 0) {
    int axis = docRx["axis"];
    if(axis == 0) { 
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

/**
 * ============================================================================
 * FONCTIONS : TÉLÉMÉTRIE JSON
 * ============================================================================
 */

void sendPositions() {
  docTx.clear();
  docTx["src"] = "mega";
  JsonArray pos = docTx["pos"].to<JsonArray>();
  pos.add(ST1.currentPosition());
  pos.add(ST2.currentPosition());
  pos.add(ST3.currentPosition());
  pos.add(ST4.currentPosition());
  serializeJson(docTx, Serial);
  Serial.println();
}

void sendLimits() {
  docTx.clear();
  docTx["src"] = "mega";
  
  // --- ÉTAT DES FINS DE COURSE (8 capteurs) ---
  JsonArray mr = docTx["mr"].to<JsonArray>();
  mr.add(digitalRead(PIN_M1_MIN) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M1_MAX) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M2_MIN) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M2_MAX) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M3_MIN) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M3_MAX) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M4_MIN) == LOW ? 1 : 0);
  mr.add(digitalRead(PIN_M4_MAX) == LOW ? 1 : 0);

  // --- ÉTAT DES ALARMES DRIVERS (ALM) ---
  JsonArray alm = docTx["alm"].to<JsonArray>();
  alm.add(digitalRead(MEGA_001MO_ALM) == HIGH ? 1 : 0); // Moteur 1
  alm.add(0);                                          // Moteur 2 (Non câblé)
  alm.add(digitalRead(MEGA_003MO_ALM) == HIGH ? 1 : 0); // Moteur 3
  alm.add(digitalRead(MEGA_004MO_ALM) == HIGH ? 1 : 0); // Moteur 4
  
  serializeJson(docTx, Serial);
  Serial.println();
}

void sendPower() {
  docTx.clear();
  docTx["src"] = "mega";
  JsonArray pwr = docTx["pwr"].to<JsonArray>();

  // Format : [V, I, P] pour chaque capteur (12 valeurs au total)
  
  if (ina1_ok) { 
    pwr.add(ina1.readBusVoltage()); pwr.add(ina1.readCurrent()); pwr.add(ina1.readPower()); 
  } else { pwr.add(0); pwr.add(0); pwr.add(0); }

  if (ina2_ok) { 
    pwr.add(ina2.readBusVoltage()); pwr.add(ina2.readCurrent()); pwr.add(ina2.readPower()); 
  } else { pwr.add(0); pwr.add(0); pwr.add(0); }

  if (ina3_ok) { 
    pwr.add(ina3.readBusVoltage()); pwr.add(ina3.readCurrent()); pwr.add(ina3.readPower()); 
  } else { pwr.add(0); pwr.add(0); pwr.add(0); }

  if (ina4_ok) { 
    pwr.add(ina4.readBusVoltage()); pwr.add(ina4.readCurrent()); pwr.add(ina4.readPower()); 
  } else { pwr.add(0); pwr.add(0); pwr.add(0); }

  serializeJson(docTx, Serial); 
  Serial.println();
}
