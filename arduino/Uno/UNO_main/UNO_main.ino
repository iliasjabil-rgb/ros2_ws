//#include <Arduino_RouterBridge.h>  // inutile sur UNO R3
//#define Serial Monitor              // surtout PAS ça sur UNO R3

#include <Wire.h>
#include <Servo.h>

#include "pins_uno.h"
#include "protocol.h"

#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include <VL53L0X.h>

// En haut du fichier
#define USE_IMU      1
#define USE_MCP9808  1
#define USE_VL53L0X  1


// -------------------- LED RGB (ruban) --------------------
#define LED_COUNT UNO_001LED_COUNT
Adafruit_NeoPixel strip(LED_COUNT, UNO_001LED_PIN, NEO_GRB + NEO_KHZ800);

// -------------------- Capteurs I2C --------------------
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
VL53L0X tof;

// -------------------- Globales --------------------
Servo servo1, servo2, servo3, servo4;
String rx;

bool imu_ok = false;
bool mcp_ok = false;
bool tof_ok = false;
bool led_effect_enabled = false;             // effet SRS actif ou non
uint32_t last_led_effect_ms = 0;
const uint16_t LED_EFFECT_PERIOD_MS = 15;    // équivalent à delay(15) mais non bloquant


// -------------------- Helpers sorties --------------------
void digitalSafeWrite(uint8_t pin, int level) {
  if (pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
  }
}

void pwmSafeWrite(uint8_t pin, int pwm) {
  if (pin >= 0) {
    pinMode(pin, OUTPUT);
    analogWrite(pin, constrain(pwm, 0, 255));
  }
}

void updateLedEffect(uint32_t now_ms) {
  if (!led_effect_enabled) return;  // effet désactivé -> on ne fait rien

  if (now_ms - last_led_effect_ms < LED_EFFECT_PERIOD_MS) {
    return; // pas encore temps de faire une nouvelle itération
  }
  last_led_effect_ms = now_ms;

  // --- Étape 1 : éteindre quelques LEDs au hasard ---
  for (int k = 0; k < 5; k++) {
    int pixelEteint = random(LED_COUNT);
    strip.setPixelColor(pixelEteint, strip.Color(0, 0, 0));
  }

  // --- Étape 2 : allumer une LED avec une couleur SRS aléatoire ---
  int pixelAllume  = random(LED_COUNT);
  int choixCouleur = random(3);  // 0,1,2

  uint8_t r = 0, g = 0, b = 0;
  if (choixCouleur == 0) {
    // Jaune
    r = 255; g = 255; b = 0;
  } else if (choixCouleur == 1) {
    // Blanc
    r = 255; g = 255; b = 255;
  } else {
    // Bleu foncé
    r = 0; g = 0; b = 255;
  }

  strip.setPixelColor(pixelAllume, strip.Color(r, g, b));
  strip.show();
}


// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);          
  
  send_json(F("{\"src\":\"uno\",\"event\":\"BOOT_UNO_MAIN_DEBUT\"}"));
  // I2C
  Wire.begin();
  Wire.setClock(400000);
  //Serial.println(F("APRES_WIRE"));
#if USE_IMU
  // ----- IMU LSM9DS1 -----
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
  // ----- Température MCP9808 (adresse mesurée : 0x1F) -----
  if (mcp9808.begin(0x1F)) {
    mcp_ok = true;
    mcp9808.setResolution(3);
    send_json(F("{\"src\":\"uno\",\"event\":\"mcp9808_temp__ok\"}"));
  } else {
    mcp_ok = false;
    send_json(F("{\"src\":\"uno\",\"event\":\"mcp9808_temp_init_failed\"}"));
  }
#endif

#if USE_VL53L0X
  // ----- Distance VL53L0X -----
  if (tof.init()) {
    tof.setTimeout(50);
    tof_ok = true;
    send_json(F("{\"src\":\"uno\",\"event\":\"vl53l0x_dist_ok\"}"));
  } else {
    tof_ok = false;
    send_json(F("{\"src\":\"uno\",\"event\":\"vl53l0x_dist_init_failed\"}"));
  }
#endif
  // ----- Sorties relais / EV -----
  digitalSafeWrite(UNO_001XR_PIN, LOW);
  digitalSafeWrite(UNO_002XR_PIN, LOW);
  digitalSafeWrite(UNO_003XR_PIN, LOW);
  digitalSafeWrite(UNO_004XR_PIN, LOW);
  digitalSafeWrite(UNO_005XR_PIN, LOW);
  digitalSafeWrite(UNO_006XR_PIN, LOW);

  // ----- Servos -----
if (UNO_001SV_PIN>=0) { servo1.attach(UNO_001SV_PIN, 500, 2500); servo1.write(90); }
if (UNO_002SV_PIN>=0) { servo2.attach(UNO_002SV_PIN, 500, 2500); servo2.write(90); }
if (UNO_003SV_PIN>=0) { servo3.attach(UNO_003SV_PIN, 500, 2500); servo3.write(89); }
if (UNO_004SV_PIN>=0) { servo4.attach(UNO_004SV_PIN, 500, 2500); servo4.write(89); }


  // ----- Ruban LED -----
  if (UNO_001LED_PIN >= 0) {
    strip.begin();
    strip.setBrightness(127);  // moyen par défaut
    strip.show();  // tout éteint
  }
  send_json(F("{\"src\":\"uno\",\"event\":\"SETUP_FIN\"}"));

  send_json(String("{\"board\":\"") + UNO_BOARD_ID +
            "\",\"fw\":\"" + UNO_FW_VERSION + "\",\"event\":\"boot\"}");
}

// -------------------- Commandes JSON --------------------
void handleCmd(const String& line) {

  // ----- Commande servos -----
if (line.indexOf("\"cmd\":\"servo\"")>=0) {
  int id = 1;
  int ang = 90;
  int p;

  p = line.indexOf("\"id\":");
  if (p >= 0) id = line.substring(p + 5).toInt();

  p = line.indexOf("\"angle\":");
  if (p >= 0) ang = line.substring(p + 8).toInt();

  int cmd_angle = ang;

  if (id == 1 || id == 2) {
    cmd_angle = constrain(ang, 0, 180);   // servos position
  } else if (id == 3 || id == 4) {
    cmd_angle = constrain(ang, 0, 180);   // servos continus, 177 = stop
  }

  if (id == 1 && UNO_001SV_PIN >= 0)      servo1.write(cmd_angle);
  else if (id == 2 && UNO_002SV_PIN >= 0) servo2.write(cmd_angle);
  else if (id == 3 && UNO_003SV_PIN >= 0) servo3.write(cmd_angle);
  else if (id == 4 && UNO_004SV_PIN >= 0) servo4.write(cmd_angle);

  send_json(String("{\"src\":\"uno\",\"event\":\"servo\",\"id\":") +
            id + ",\"angle_cmd\":" + cmd_angle + "}");
}


  else if (line.indexOf("\"cmd\":\"relay\"") >= 0) {
    int id = 1, st = 0;
    int p;

    p = line.indexOf("\"id\":");    if (p >= 0) id = line.substring(p + 5).toInt();
    p = line.indexOf("\"state\":"); if (p >= 0) st = line.substring(p + 8).toInt();

    uint8_t pin = (id == 1) ? UNO_001XR_PIN :
                  (id == 2) ? UNO_002XR_PIN :
                  (id == 3) ? UNO_003XR_PIN :
                  (id == 4) ? UNO_004XR_PIN :
                  (id == 5) ? UNO_005XR_PIN :
                  (id == 6) ? UNO_006XR_PIN : 0;

    digitalSafeWrite(pin, st ? HIGH : LOW);

    send_json(String("{\"src\":\"uno\",\"event\":\"relay\",\"id\":") +
              id + ",\"state\":" + st + "}");
  }


  else if (line.indexOf("\"cmd\":\"led\"") >= 0) {
    int r = 0, g = 0, b = 0, a = 255; // a = luminosité par défaut (max)
    int p;

    p = line.indexOf("\"r\":"); if (p >= 0) r = line.substring(p + 4).toInt();
    p = line.indexOf("\"g\":"); if (p >= 0) g = line.substring(p + 4).toInt();
    p = line.indexOf("\"b\":"); if (p >= 0) b = line.substring(p + 4).toInt();
    p = line.indexOf("\"a\":"); if (p >= 0) a = line.substring(p + 4).toInt();

    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);
    a = constrain(a, 0, 255);  // 0 = éteint, 255 = plein pot

        if (UNO_001LED_PIN >= 0) {
      // Un ordre LED “statique” coupe l’effet scintillant
      led_effect_enabled = false;

      strip.setBrightness(a);        // luminosité globale
      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
      }
      strip.show();
    }


    send_json(String("{\"src\":\"uno\",\"event\":\"led\",\"r\":") +
              r + ",\"g\":" + g + ",\"b\":" + b + ",\"a\":" + a + "}");
  }
    else if (line.indexOf("\"cmd\":\"led_effect\"") >= 0) {
    int st = 0;
    int p = line.indexOf("\"state\":");
    if (p >= 0) {
      st = line.substring(p + 8).toInt();
    }

    led_effect_enabled = (st != 0);

    if (led_effect_enabled) {
      // --- CORRECTION 1 : On force la luminosité au max ---
      // Sinon, si l'intensité était à 0 avant, l'effet est invisible !
      strip.setBrightness(255); 
    } else {
      // Quand on coupe l'effet, on éteint tout proprement
      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, 0);
      }
      strip.show();
    }

    send_json(String("{\"src\":\"uno\",\"event\":\"led_effect\",\"state\":") +
              st + "}");
  }

}

// -------------------- LOOP --------------------
void loop() {
  // Lecture des commandes (si tu tapes quelque chose dans le moniteur série)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCmd(rx);
      rx = "";
    } else if (c != '\r') {
      rx += c;
    }
  }
  
  // Télémétrie toutes les 250 ms
  static uint32_t t0 = 0;
  uint32_t ms = millis();

  if (ms - t0 > 250) {
    t0 = ms;

    // DEBUG pour vérifier que le loop tourne bien
    //Serial.println(F("LOOP_TICK"));

    // --- IMU ---
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;

    if (imu_ok) {
      sensors_event_t a_event, g_event, m_event, temp_event;
      imu.getEvent(&a_event, &g_event, &m_event, &temp_event);

      ax = a_event.acceleration.x;
      ay = a_event.acceleration.y;
      az = a_event.acceleration.z;

      gx = g_event.gyro.x;
      gy = g_event.gyro.y;
      gz = g_event.gyro.z;

      mx = m_event.magnetic.x; 
      my = m_event.magnetic.y;
      mz = m_event.magnetic.z;
    }

    // --- Température ---
    float temp_c = 0.0f;
    if (mcp_ok) {
      temp_c = mcp9808.readTempC();
    }

    // --- Distance ---
    uint16_t dist_mm = 0;
    if (tof_ok) {
      dist_mm = tof.readRangeSingleMillimeters();
      if (tof.timeoutOccurred()) {
        send_json(F("{\"src\":\"uno\",\"event\":\"VL53_dist_TIMEOUT\"}"));
      }
    }

// JSON télémétrie, construit "à la main" pour éviter les gros String
Serial.print(F("{\"src\":\"uno\""));

// États capteurs
Serial.print(F(",\"imu_ok\":"));  Serial.print(imu_ok ? 1 : 0);
Serial.print(F(",\"mcp_ok\":"));  Serial.print(mcp_ok ? 1 : 0);
Serial.print(F(",\"tof_ok\":"));  Serial.print(tof_ok ? 1 : 0);

// Température + distance
Serial.print(F(",\"temp_c\":"));  Serial.print(temp_c, 2);
Serial.print(F(",\"dist_mm\":")); Serial.print(dist_mm);

// IMU linéaire
Serial.print(F(",\"ax\":"));      Serial.print(ax, 3);
Serial.print(F(",\"ay\":"));      Serial.print(ay, 3);
Serial.print(F(",\"az\":"));      Serial.print(az, 3);

// IMU gyro
Serial.print(F(",\"gx\":"));      Serial.print(gx, 3);
Serial.print(F(",\"gy\":"));      Serial.print(gy, 3);
Serial.print(F(",\"gz\":"));      Serial.print(gz, 3);

// (optionnel) champ magnétique
Serial.print(F(",\"mx\":"));      Serial.print(mx, 3);
Serial.print(F(",\"my\":"));      Serial.print(my, 3);
Serial.print(F(",\"mz\":"));      Serial.print(mz, 3);



// Fin de l’objet JSON
Serial.println('}');


  }
    // Mise à jour non bloquante de l’animation SRS
  updateLedEffect(ms);
}
