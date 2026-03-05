/**
* ============================================================================
* FICHIER : UNO_main.ino (V4.1 - Avec Calibration IMU)
* ROLE    : Programme principal de l'Arduino Uno.
* - Gestion des actionneurs secondaires (Servos, Relais, LED)
* - Lecture des capteurs I2C (IMU, Température, Distance ToF)
* - Parsing JSON "manuel" optimisé pour économiser la RAM de la Uno.
* - Calibrage automatique du gyroscope au démarrage.
* ============================================================================
*/
 
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include <VL53L0X.h>
 
#include "pins_uno.h"
#include "protocol.h"
 
/**
* ============================================================================
* CONFIGURATION MATÉRIELLE (Drapeaux d'activation)
* ============================================================================
*/
#define USE_IMU      1
#define USE_MCP9808  1
#define USE_VL53L0X  1
 
const uint32_t TELEMETRY_INTERVAL_MS = 250; // Télémétrie à 4 Hz
 
/**
* ============================================================================
* VARIABLES GLOBALES ET OBJETS
* ============================================================================
*/
// --- Ruban LED RGB ---
#define LED_COUNT UNO_001LED_COUNT
Adafruit_NeoPixel strip(LED_COUNT, UNO_001LED_PIN, NEO_GRB + NEO_KHZ800);
 
// --- Capteurs I2C ---
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
VL53L0X tof;
 
// --- Actionneurs et Communication ---
Servo servo1, servo2, servo3, servo4;
String rx; // Buffer de réception série
 
// --- États du système ---
bool imu_ok = false;
bool mcp_ok = false;
bool tof_ok = false;

// --- Offsets de calibration du Gyroscope ---
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
 
/**
* ============================================================================
* FONCTIONS UTILITAIRES
* ============================================================================
*/
 
// Écrit proprement sur une sortie digitale si la broche est valide
void digitalSafeWrite(int8_t pin, int level) {
  if (pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
  }
}
 
/**
* ============================================================================
* SETUP : INITIALISATION
* ============================================================================
*/
void setup() {
  Serial.begin(115200);          
  send_json(F("{\"src\":\"uno\",\"event\":\"BOOT_UNO_MAIN_DEBUT\"}"));
  // Pré-allocation pour limiter la fragmentation RAM avec la String de réception
  rx.reserve(128);
  // --- Initialisation Fin de course ---
  pinMode(UNO_001FC_PIN, INPUT_PULLUP);
 
  // --- Initialisation I2C Haut Débit ---
  Wire.begin();
  Wire.setClock(400000);
 
#if USE_IMU
  if (!imu.begin()) {
    imu_ok = false;
    send_json(F("{\"src\":\"uno\",\"event\":\"imu_init_failed\"}"));
  } else {
    imu_ok = true;
    send_json(F("{\"src\":\"uno\",\"event\":\"imu_ok_starting_calibration\"}"));
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);

    // --- CALIBRATION DU GYROSCOPE (Prend environ 1 à 2 secondes) ---
    // NE PAS BOUGER LE CAPTEUR PENDANT CE TEMPS
    const int NUM_SAMPLES = 200;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    sensors_event_t a_evt, g_evt, m_evt, t_evt;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        imu.getEvent(&a_evt, &g_evt, &m_evt, &t_evt);
        sum_x += g_evt.gyro.x;
        sum_y += g_evt.gyro.y;
        sum_z += g_evt.gyro.z;
        delay(5);
    }
    
    // Calcul de la moyenne du bruit au repos
    gyro_offset_x = sum_x / NUM_SAMPLES;
    gyro_offset_y = sum_y / NUM_SAMPLES;
    gyro_offset_z = sum_z / NUM_SAMPLES;
    
    send_json(F("{\"src\":\"uno\",\"event\":\"imu_calibrated\"}"));
  }
#endif
 
#if USE_MCP9808
  if (mcp9808.begin(0x1F)) { // Adresse par défaut
    mcp_ok = true;
    mcp9808.setResolution(3);
    send_json(F("{\"src\":\"uno\",\"event\":\"mcp9808_temp_ok\"}"));
  } else {
    mcp_ok = false;
    send_json(F("{\"src\":\"uno\",\"event\":\"mcp9808_temp_init_failed\"}"));
  }
#endif
 
#if USE_VL53L0X
  if (tof.init()) {
    tof.setTimeout(50);
    tof_ok = true;
    send_json(F("{\"src\":\"uno\",\"event\":\"vl53l0x_dist_ok\"}"));
  } else {
    tof_ok = false;
    send_json(F("{\"src\":\"uno\",\"event\":\"vl53l0x_dist_init_failed\"}"));
  }
#endif
 
  // --- Initialisation Relais ---
  digitalSafeWrite(UNO_001XR_PIN, LOW);  // Ruban LED Blanc coupé
  digitalSafeWrite(UNO_002XR_PIN, HIGH); // Relais Ruban LED RGB activé pour l'allumage
  digitalSafeWrite(UNO_003XR_PIN, LOW);
  digitalSafeWrite(UNO_004XR_PIN, LOW);
  digitalSafeWrite(UNO_005XR_PIN, LOW);
  digitalSafeWrite(UNO_006XR_PIN, LOW);
 
  // --- Initialisation Servos ---
  if (UNO_001SV_PIN >= 0) { servo1.attach(UNO_001SV_PIN, 500, 2500); servo1.write(90); }
  if (UNO_002SV_PIN >= 0) { servo2.attach(UNO_002SV_PIN, 500, 2500); servo2.write(90); }
  if (UNO_003SV_PIN >= 0) { servo3.attach(UNO_003SV_PIN, 500, 2500); servo3.write(89); }
  if (UNO_004SV_PIN >= 0) { servo4.attach(UNO_004SV_PIN, 500, 2500); servo4.write(89); }
 
  // --- Initialisation Ruban LED ---
  if (UNO_001LED_PIN >= 0) {
    delay(10);
    strip.begin();
    strip.setBrightness(30);
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(255, 255, 255));
    }
    strip.show(); 
  }
 
  send_json(String("{\"board\":\"") + UNO_BOARD_ID + "\",\"fw\":\"" + UNO_FW_VERSION + "\",\"event\":\"boot_ready\"}");
}
 
/**
* ============================================================================
* FONCTION : PARSING DES COMMANDES (Optimisé RAM)
* ============================================================================
*/
void handleCmd(const String& line) {
 
  // ----------------------------------------------------------------
  // COMMANDE : SERVO
  // ----------------------------------------------------------------
  if (line.indexOf("\"cmd\":\"servo\"") >= 0) {
    int id = 1, ang = 90, p;
 
    p = line.indexOf("\"id\":");    if (p >= 0) id = line.substring(p + 5).toInt();
    p = line.indexOf("\"angle\":"); if (p >= 0) ang = line.substring(p + 8).toInt();
 
    int cmd_angle = constrain(ang, 0, 180);
 
    if      (id == 1 && UNO_001SV_PIN >= 0) servo1.write(cmd_angle);
    else if (id == 2 && UNO_002SV_PIN >= 0) servo2.write(cmd_angle);
    else if (id == 3 && UNO_003SV_PIN >= 0) servo3.write(cmd_angle);
    else if (id == 4 && UNO_004SV_PIN >= 0) servo4.write(cmd_angle);
 
    send_json(String("{\"src\":\"uno\",\"event\":\"servo\",\"id\":") + id + ",\"angle_cmd\":" + cmd_angle + "}");
  }
 
  // ----------------------------------------------------------------
  // COMMANDE : RELAY
  // ----------------------------------------------------------------
  else if (line.indexOf("\"cmd\":\"relay\"") >= 0) {
    int id = 1, st = 0, p;
 
    p = line.indexOf("\"id\":");    if (p >= 0) id = line.substring(p + 5).toInt();
    p = line.indexOf("\"state\":"); if (p >= 0) st = line.substring(p + 8).toInt();
 
    uint8_t pin = (id == 1) ? UNO_001XR_PIN :
                  (id == 2) ? UNO_002XR_PIN :
                  (id == 3) ? UNO_003XR_PIN :
                  (id == 4) ? UNO_004XR_PIN :
                  (id == 5) ? UNO_005XR_PIN :
                  (id == 6) ? UNO_006XR_PIN : 0;
 
    digitalSafeWrite(pin, st ? HIGH : LOW);
    send_json(String("{\"src\":\"uno\",\"event\":\"relay\",\"id\":") + id + ",\"state\":" + st + "}");
  }
 
  // ----------------------------------------------------------------
  // COMMANDE : LED (Couleur Fixe & Pilotage via ROS 2)
  // ----------------------------------------------------------------
  else if (line.indexOf("\"cmd\":\"led\"") >= 0) {
    int r = 0, g = 0, b = 0, a = 80, id = -1, p;
 
    p = line.indexOf("\"id\":"); if (p >= 0) id = line.substring(p + 5).toInt();
    p = line.indexOf("\"r\":");  if (p >= 0) r  = line.substring(p + 4).toInt();
    p = line.indexOf("\"g\":");  if (p >= 0) g  = line.substring(p + 4).toInt();
    p = line.indexOf("\"b\":");  if (p >= 0) b  = line.substring(p + 4).toInt();
    p = line.indexOf("\"a\":");  if (p >= 0) a  = line.substring(p + 4).toInt();
 
    r = constrain(r, 0, 255); g = constrain(g, 0, 255);
    b = constrain(b, 0, 255); a = constrain(a, 0, 255);
 
    if (UNO_001LED_PIN >= 0) {
      strip.setBrightness(a); 
      if (id >= 0 && id < LED_COUNT) {
        strip.setPixelColor(id, strip.Color(r, g, b));
      } else {
        for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, strip.Color(r, g, b));
      }
      strip.show();
    }
  }
}
 
/**
* ============================================================================
* BOUCLE PRINCIPALE
* ============================================================================
*/
void loop() {
  // --- 1. Lecture asynchrone du port Série ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCmd(rx);
      rx = "";
    } else if (c != '\r') {
      rx += c;
    }
  }
 
  // --- 2. SÉCURITÉ FIN DE COURSE (Servo 4 - Serrage Pince) ---
  if (digitalRead(UNO_001FC_PIN) == LOW) {
    if (servo4.read() > 90) {
      servo4.write(90);
    }
  }
  // --- 3. Télémétrie périodique ---
  static uint32_t t0 = 0;
  uint32_t ms = millis();
 
  if (ms - t0 > TELEMETRY_INTERVAL_MS) {
    t0 = ms;
 
    // Variables par défaut
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    float temp_c = 0.0f;
    uint16_t dist_mm = 0;
 
    // Lecture IMU
    if (imu_ok) {
      sensors_event_t a_event, g_event, m_event, temp_event;
      imu.getEvent(&a_event, &g_event, &m_event, &temp_event);
 
      ax = a_event.acceleration.x; ay = a_event.acceleration.y; az = a_event.acceleration.z;
      
      // Soustraction de l'offset et Conversion en Radians par seconde (rad/s)
      gx = (g_event.gyro.x - gyro_offset_x) * 0.0174533f;         
      gy = (g_event.gyro.y - gyro_offset_y) * 0.0174533f;         
      gz = (g_event.gyro.z - gyro_offset_z) * 0.0174533f;

      mx = m_event.magnetic.x;     my = m_event.magnetic.y;     mz = m_event.magnetic.z;
    }
 
    // Lecture Température
    if (mcp_ok) temp_c = mcp9808.readTempC();
 
    // Lecture ToF (Laser)
    if (tof_ok) {
      dist_mm = tof.readRangeSingleMillimeters();
      if (tof.timeoutOccurred()) send_json(F("{\"src\":\"uno\",\"event\":\"VL53_dist_TIMEOUT\"}"));
    }
 
    // ------------------------------------------------------------------
    // FORMATAGE MANUEL DU JSON
    // ------------------------------------------------------------------
    Serial.print(F("{\"src\":\"uno\""));
 
    Serial.print(F(",\"imu_ok\":")); Serial.print(imu_ok ? 1 : 0);
    Serial.print(F(",\"mcp_ok\":")); Serial.print(mcp_ok ? 1 : 0);
    Serial.print(F(",\"tof_ok\":")); Serial.print(tof_ok ? 1 : 0);
 
    Serial.print(F(",\"temp_c\":"));  Serial.print(temp_c, 2);
    Serial.print(F(",\"dist_mm\":")); Serial.print(dist_mm);
 
    Serial.print(F(",\"ax\":")); Serial.print(ax, 3);
    Serial.print(F(",\"ay\":")); Serial.print(ay, 3);
    Serial.print(F(",\"az\":")); Serial.print(az, 3);
 
    // Précision augmentée à 4 décimales pour les radians
    Serial.print(F(",\"gx\":")); Serial.print(gx, 4);
    Serial.print(F(",\"gy\":")); Serial.print(gy, 4);
    Serial.print(F(",\"gz\":")); Serial.print(gz, 4);
 
    Serial.print(F(",\"mx\":")); Serial.print(mx, 3);
    Serial.print(F(",\"my\":")); Serial.print(my, 3);
    Serial.print(F(",\"mz\":")); Serial.print(mz, 3);
 
    Serial.println('}'); 
  }
}