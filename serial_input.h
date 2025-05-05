#pragma once

#include <Arduino.h>

extern bool switch1StableState;
extern bool switch2StableState;
extern bool pedalStableState;
extern int currentServoAngle;


void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int equalsIndex = input.indexOf('=');
    if (equalsIndex == -1) {
      Serial.println("❌ Формат неправильний. Пиши: ім'я = значення");
      return;
    }

    String varName = input.substring(0, equalsIndex);
    String varValue = input.substring(equalsIndex + 1);
    varName.trim();
    varValue.trim();

    int value = varValue.toInt();

    if (varName == "switch1") {
      switch1StableState = value;
      Serial.println("✅ switch1 оновлено");
    }
    else if (varName == "switch2") {
      switch2StableState = value;
      Serial.println("✅ switch2 оновлено");
    }
    else if (varName == "pedal") {
      pedalStableState = value;
      Serial.println("✅ pedal оновлено");
    }
    else if (varName == "Servo") {
      if (value >= 0 && value <= 180) {
        currentServoAngle = value;
        Serial.println("✅ Servo оновлено");
      } else {
        Serial.println("⚠️ Значення Servo має бути від 0 до 180");
      }
    }
    else {
      Serial.println("❌ Невідома змінна");
    }

    // Вивести всі значення для наочності
    Serial.print("switch1: "); Serial.println(switch1StableState);
    Serial.print("switch2: "); Serial.println(switch2StableState);
    Serial.print("pedal: "); Serial.println(pedalStableState);
    Serial.print("Servo: "); Serial.println(currentServoAngle);
    Serial.println("----");
  }
}
