/**
 * ============================================================================
 * FICHIER : UNO_main.ino (V3 - Clean)
 * ROLE    : Programme principal de l'Arduino Uno.
 * - Gestion des actionneurs secondaires (Servos, Relais, LED)
 * - Lecture des capteurs I2C (IMU, Température, Distance ToF)
 * - Parsing JSON "manuel" optimisé pour économiser la RAM de la Uno.
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
const uint16_t LED_EFFECT_PERIOD_MS  = 15;  // Vitesse de l'effet SRS

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

// --- Gestion de l'effet visuel SRS ---
bool led_effect_enabled = false;
uint32_t last_led_effect_ms = 0;


/**
 * ============================================================================
 * FONCTIONS UTILITAIRES (HELPERS)
 * ============================================================================
 */

// Écrit proprement sur une sortie digitale si la broche est valide
void digitalSafeWrite(int8_t pin, int level) {
  if (pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
  }
}

// Génère l'animation visuelle aléatoire (Effet SRS) de façon non-bloquante
void updateLedEffect(uint32_t now_ms) {
  if (!led_effect_enabled) return; 

  if (now_ms - last_led_effect_ms < LED_EFFECT_PERIOD_MS) {
    return; // Temporisation non atteinte
  }
  last_led_effect_ms = now_ms;

  // Étape 1 : Éteindre 5 LEDs au hasard
  for (int k = 0; k < 5; k++) {
    int pixelEteint = random(LED_COUNT);
    strip.setPixelColor(pixelEteint, 0, 0, 0); // Noir
  }

  // Étape 2 : Allumer 1 LED avec une couleur aléatoire de la charte SRS
  int pixelAllume  = random(LED_COUNT);
  int choixCouleur = random(3); 

  uint8_t r = 0, g = 0, b = 0;
  if (choixCouleur == 0) {
    r = 255; g = 255; b = 0;   // Jaune
  } else if (choixCouleur == 1) {
    r = 255; g = 255; b = 255; // Blanc
  } else {
    r = 0;   g = 0;   b = 255; // Bleu foncé
  }

  strip.setPixelColor(pixelAllume, strip.Color(r, g, b));
  strip.show();
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
    send_json(F("{\"src\":\"uno\",\"event\":\"imu_ok\"}"));
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
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
    // Pause de sécurité (10ms) pour laisser le relais claquer avant l'envoi des données
    delay(10);
    strip.begin();
    strip.setBrightness(30);
    
    // On applique la couleur Blanche (255, 255, 255) à chaque LED au démarrage
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

    int cmd_angle = ang;

    // Servos 1 & 2 : Positionnement classique (0-180)
    // Servos 3 & 4 : Rotation continue (90 = stop absolu, selon étalonnage)
    cmd_angle = constrain(ang, 0, 180);

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
  // COMMANDE : LED_EFFECT (Animation SRS)
  // ----------------------------------------------------------------
  else if (line.indexOf("\"cmd\":\"led_effect\"") >= 0) {
    int st = 0, p;
    
    p = line.indexOf("\"state\":"); 
    if (p >= 0) st = line.substring(p + 8).toInt();

    led_effect_enabled = (st != 0);

    if (led_effect_enabled) {
      strip.setBrightness(255); // Force la luminosité max pour l'effet
    } else {
      for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, 0);
      strip.show(); // Extinction propre
    }
    send_json(String("{\"src\":\"uno\",\"event\":\"led_effect\",\"state\":") + st + "}");
  }

  // ----------------------------------------------------------------
  // COMMANDE : LED (Couleur Fixe)
  // ----------------------------------------------------------------
  else if (line.indexOf("\"cmd\":\"led\"") >= 0) {
    // Valeurs par défaut : Blanc (255,255,255), Intensité 100, ID -1 (Toutes les LEDs)
    int r = 255, g = 255, b = 255, a = 80, id = -1, p;

    p = line.indexOf("\"id\":"); if (p >= 0) id = line.substring(p + 5).toInt();
    p = line.indexOf("\"r\":");  if (p >= 0) r  = line.substring(p + 4).toInt();
    p = line.indexOf("\"g\":");  if (p >= 0) g  = line.substring(p + 4).toInt();
    p = line.indexOf("\"b\":");  if (p >= 0) b  = line.substring(p + 4).toInt();
    p = line.indexOf("\"a\":");  if (p >= 0) a  = line.substring(p + 4).toInt(); // Luminosité

    r = constrain(r, 0, 255); g = constrain(g, 0, 255);
    b = constrain(b, 0, 255); a = constrain(a, 0, 255);

    if (UNO_001LED_PIN >= 0) {
      led_effect_enabled = false; // Un ordre statique coupe l'animation SRS
      strip.setBrightness(a); 
      
      if (id >= 0 && id < LED_COUNT) {
        strip.setPixelColor(id, strip.Color(r, g, b));
      } else {
        for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, strip.Color(r, g, b));
      }
      strip.show();
    }
    send_json(String("{\"src\":\"uno\",\"event\":\"led\",\"id\":") + id + ",\"r\":" + r + ",\"g\":" + g + ",\"b\":" + b + ",\"a\":" + a + "}");
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
      gx = g_event.gyro.x;         gy = g_event.gyro.y;         gz = g_event.gyro.z;
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
    // FORMATAGE MANUEL DU JSON (Évite d'exploser la RAM avec un String)
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

    Serial.print(F(",\"gx\":")); Serial.print(gx, 3);
    Serial.print(F(",\"gy\":")); Serial.print(gy, 3);
    Serial.print(F(",\"gz\":")); Serial.print(gz, 3);

    Serial.print(F(",\"mx\":")); Serial.print(mx, 3);
    Serial.print(F(",\"my\":")); Serial.print(my, 3);
    Serial.print(F(",\"mz\":")); Serial.print(mz, 3);

    Serial.println('}'); // Clôture le JSON
  }

  // --- 4. Mise à jour de l'animation LED (Non-bloquant) ---
  updateLedEffect(ms);
}
