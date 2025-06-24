// ESP32 Encoder Reader - Envia valores brutos dos encoders via Serial (USB)
// Para integração com ROS2 joint_state_publisher
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Encoder.h>

// Defina os pinos dos encoders conforme seu hardware
#define ENC_FL_A 33
#define ENC_FL_B 25
#define ENC_FR_A 23
#define ENC_FR_B 22
#define ENC_RL_A 26
#define ENC_RL_B 27
#define ENC_RR_A 18
#define ENC_RR_B 19

ESP32Encoder encoder_FL;
ESP32Encoder encoder_FR;
ESP32Encoder encoder_RL;
ESP32Encoder encoder_RR;

unsigned long lastSend = 0;
const unsigned long sendInterval = 20; // ms

void setup() {
  Serial.begin(115200);
  encoder_FL.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  encoder_FR.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  encoder_RL.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  encoder_RR.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  encoder_FL.clearCount();
  encoder_FR.clearCount();
  encoder_RL.clearCount();
  encoder_RR.clearCount();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= sendInterval) {
    lastSend = now;
    StaticJsonDocument<128> doc;
    doc["enc_fl"] = encoder_FL.getCount();
    doc["enc_fr"] = encoder_FR.getCount();
    doc["enc_rl"] = encoder_RL.getCount();
    doc["enc_rr"] = encoder_RR.getCount();
    serializeJson(doc, Serial);
    Serial.println();
  }
}
