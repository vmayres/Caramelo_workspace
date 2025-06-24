// ESP32_PWM_writer.ino
// Recebe comandos PWM via Serial (JSON) e aplica nos motores
#include <Arduino.h>
#include <ArduinoJson.h>
#define PIN_FL 32
#define PIN_FR 21
#define PIN_RL 13
#define PIN_RR 2
#define PWM_FREQ 5000
#define PWM_RESOLUTION 10 // 0-1023
void setup() {
  Serial.begin(9600);
  ledcAttachPin(PIN_FL, 0);
  ledcAttachPin(PIN_FR, 1);
  ledcAttachPin(PIN_RL, 2);
  ledcAttachPin(PIN_RR, 3);
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(0, 512);
  ledcWrite(1, 512);
  ledcWrite(2, 512);
  ledcWrite(3, 512);
}
void loop() {
  static String inputString = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      StaticJsonDocument<128> doc;
      DeserializationError error = deserializeJson(doc, inputString);
      if (!error) {
        int pwm_fl = doc["pwm_fl"] | 512;
        int pwm_fr = doc["pwm_fr"] | 512;
        int pwm_rl = doc["pwm_rl"] | 512;
        int pwm_rr = doc["pwm_rr"] | 512;
        ledcWrite(0, pwm_fl);
        ledcWrite(1, pwm_fr);
        ledcWrite(2, pwm_rl);
        ledcWrite(3, pwm_rr);
      }
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}
