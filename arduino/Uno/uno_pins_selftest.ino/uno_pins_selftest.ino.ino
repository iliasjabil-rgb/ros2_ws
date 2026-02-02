#include "pins_uno.h"

void setup() {
  Serial.begin(115200);
  while(!Serial){}

  // Sorties
  pinMode(UNO_001XR_PIN, OUTPUT);
  pinMode(UNO_002XR_PIN, OUTPUT);
  pinMode(UNO_003XR_PIN, OUTPUT);
  pinMode(UNO_004XR_PIN, OUTPUT);
  pinMode(UNO_005XR_PIN, OUTPUT);
  pinMode(UNO_006XR_PIN, OUTPUT);
  pinMode(UNO_007XR_PIN, OUTPUT);

  pinMode(UNO_001EV_PIN, OUTPUT);
  pinMode(UNO_002EV_PIN, OUTPUT);
  pinMode(UNO_003EV_PIN, OUTPUT);

  // DC driver
  pinMode(UNO_DM_M1_IN1, OUTPUT);
  pinMode(UNO_DM_M1_IN2, OUTPUT);
  pinMode(UNO_DM_M1_PWM, OUTPUT);
  pinMode(UNO_DM_M2_IN1, OUTPUT);
  pinMode(UNO_DM_M2_IN2, OUTPUT);
  pinMode(UNO_DM_M2_PWM, OUTPUT);

  Serial.println(F("{\"board\":\"002UC_ARDUINO_UNO\",\"fw\":\"1.0\",\"event\":\"boot\"}"));
  Serial.println(F("# === UNO pin map ==="));
  Serial.print(F("Relais 001..007 (D2..D8) -> ")); Serial.print(UNO_001XR_PIN); Serial.print(",");
  Serial.print(UNO_002XR_PIN); Serial.print(","); Serial.print(UNO_003XR_PIN); Serial.print(",");
  Serial.print(UNO_004XR_PIN); Serial.print(","); Serial.print(UNO_005XR_PIN); Serial.print(",");
  Serial.print(UNO_006XR_PIN); Serial.print(","); Serial.println(UNO_007XR_PIN);

  Serial.print(F("EV 001..003 (D9..D11 PWM) -> "));
  Serial.print(UNO_001EV_PIN); Serial.print(","); Serial.print(UNO_002EV_PIN);
  Serial.print(","); Serial.println(UNO_003EV_PIN);

  Serial.println(F("ADC A0..A5 = 001BL,002BL,003BL,001CD,001CT,SPARE"));
}

void loop() {
  static uint32_t t0=0;
  if (millis()-t0>300) {
    t0=millis();
    int a0=analogRead(UNO_001BL_ADC);
    int a1=analogRead(UNO_002BL_ADC);
    int a2=analogRead(UNO_003BL_ADC);
    int cd=analogRead(UNO_001CD_ADC);
    int ct=analogRead(UNO_001CT_ADC);

    Serial.print(F("{\"src\":\"uno\",\"a0\":")); Serial.print(a0);
    Serial.print(F(",\"a1\":")); Serial.print(a1);
    Serial.print(F(",\"a2\":")); Serial.print(a2);
    Serial.print(F(",\"dist\":")); Serial.print(cd);
    Serial.print(F(",\"temp\":")); Serial.print(ct);
    Serial.println(F("}"));
  }
}
