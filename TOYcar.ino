// wemos esp32-s2 mini board
#pragma once
#include <Arduino.h>
#include "esp32-hal-ledc.h"

#define DEBUG_SERIAL
 #ifdef DEBUG_SERIAL
   #define DEBUG_PRINTLN(x) Serial.println(x)
   #define DEBUG_PRINT(x)   Serial.print(x)
   #include <serial_input.h>
 #else
   #define DEBUG_PRINTLN(x)
   #define DEBUG_PRINT(x)
 #endif

//
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// Налаштування Wi-Fi
const char* ssid = "MasterNET";
const char* password = "ZakvaskA600";

// Вхідні піни
#define PIN_SWITCH1 3
#define PIN_SWITCH2 5
#define PIN_PEDAL   7

// Вихідні піни
#define PIN_SERVO_POWER 16
#define PIN_SERVO_PWM   18
#define PIN_HEADLIGHT   33

#define PIN_MOTOR_IN1   35
#define PIN_MOTOR_IN2   37
//#define PIN_MOTOR_EN    39

//Веб керування
unsigned long lastWebCommandTime = 0;

// Налаштування серво
int servoMinAngle = 45;
int servoMaxAngle = 135;
int neutralServoAngle = 90; // "нульовий" кут
enum ServoMode { SERVO_SOFT, SERVO_HARD, SERVO_HOLD_ZERO };
ServoMode servoMode = SERVO_SOFT;  // тип режиму
ServoMode previousServoMode = SERVO_SOFT;  // для збереження попереднього режиму

// Двигун
int webMotorSpeed = 0;    // Задана швидкість від веб-інтерфейсу (від 0 до 100)
int currentSpeed = 0;  // Поточна швидкість мотора
int targetSpeed =0;
int motorMaxSpeed = 100;       // Максимальна швидкість (100% = 255 для PWM)
int accelerationRate = 4;   // Крок нарощення ШИМ
unsigned long motorUpdateInterval=5; // Інтервал нарощення ШИМ, кожні х мілісекунд
bool forwardflag = 0;
bool reverseflag = 0;
bool stopflag = 1;
unsigned long lastMotorUpdate=0;

#define PWM_FREQ 5000     // 5 kHz (оптимально для моторів)
#define PWM_RESOLUTION 8  // 8 біт (значення 0-255)

//unsigned long previousMillis = 0;

//керування
bool pedalStableState;
bool switch1StableState;
bool switch2StableState;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 100;
bool headlightOn = false;


// Servo керування
Servo steeringServo;
int currentServoAngle = 90;
int lastServoAngle = -1;
bool lastServoPowerState = false;
bool servoEnabled = false;
int previousServoAngle = 90;
unsigned long servoTurnStartTime = 0;
unsigned long servoTurnDuration = 0;

// Діагностика
int lastReportedSpeed = -1;
bool lastReverseState = false;

//web керування
bool motorControlIntercepted = false;  // Інтерцепція управління мотором через веб
unsigned long webControlTimeout = 2000;  // Таймаут для веб-керування мотором

// Ініціалізуємо сервер на порту 80
AsyncWebServer server(80);

// Ініціалізуємо WebSocket на шляху /ws
AsyncWebSocket ws("/ws");

    //серво
void enableServo() {
  digitalWrite(PIN_SERVO_POWER, HIGH);
  servoEnabled = true;
  DEBUG_PRINTLN("Серва вкл");
 }

void disableServo() {
  digitalWrite(PIN_SERVO_POWER, LOW);
  servoEnabled = false;
  DEBUG_PRINTLN("Серва викл");
 }
//
void moveServoTo() {
  int angle = currentServoAngle;
  if (angle < servoMinAngle) angle = servoMinAngle;
  if (angle > servoMaxAngle) angle = servoMaxAngle;

  if (servoMode == SERVO_HARD || servoMode == SERVO_HOLD_ZERO) {
    enableServo();
    steeringServo.write(angle);
    DEBUG_PRINT("HARD кут серво: ");
    DEBUG_PRINTLN(angle);
  } else if (servoMode == SERVO_SOFT) {
    int delta = abs(currentServoAngle - lastServoAngle);
    servoTurnDuration = (unsigned long)(delta * 1.83) + 100; // мс
    servoTurnStartTime = millis();

    enableServo();
    steeringServo.write(angle);
  }

  lastServoAngle = currentServoAngle;
 }
//
 void updateServo() {
  if (servoMode == SERVO_SOFT && servoEnabled) {
    if (millis() - servoTurnStartTime > servoTurnDuration) {
      disableServo();  // вимикає живлення серви

          DEBUG_PRINT("SOFT кут серво: ");
          DEBUG_PRINTLN(currentServoAngle);
          DEBUG_PRINT("час повороту, мс: ");
          DEBUG_PRINTLN(servoTurnDuration);
          DEBUG_PRINTLN("Серво вимкнено після повороту");
    }
  }
 }

void handleWebServoControl() {
  // Якщо включено SERVO_HOLD_ZERO, завжди форсувати нейтральний кут
  if (servoMode == SERVO_HOLD_ZERO) {
    if (currentServoAngle != neutralServoAngle) {
      currentServoAngle = neutralServoAngle;
      moveServoTo();
    }
  } else {
    // Для інших режимів викликаємо переміщення тільки при зміні кута
    if (currentServoAngle != lastServoAngle) {
      moveServoTo();
    }
  }
 }
//
    //Мотор

void MotorForward(){
  if (reverseflag == 1){  // Якщо напрямок змінився
        stopMotor();  // Зупиняємо мотор при зміні напрямку
        delay(500);   // Невелика пауза перед зміною напрямку
  }
  forwardflag = 1;
  reverseflag = 0;
  stopflag = 0;
 }
//
void MotorReverse(){
  if (forwardflag == 1){  // Якщо напрямок змінився
        stopMotor();  // Зупиняємо мотор при зміні напрямку
        delay(500);   // Невелика пауза перед зміною напрямку
  }
  forwardflag = 0;
  reverseflag = 1;
  stopflag = 0;
  }
//
void stopMotor(){
  ledcWrite(PIN_MOTOR_IN1, 0);  //викл вперед
  ledcWrite(PIN_MOTOR_IN2, 0);  //викл назад
  currentSpeed = 0;           //поточна швидкість 0
  targetSpeed = 0;
    if (stopflag == 0) {
      DEBUG_PRINTLN("СТОП");
      stopflag = 1;
      forwardflag = 0;
      reverseflag = 0;
  }
 }
//
void MotorUpdate(){
     if (millis() - lastMotorUpdate >= motorUpdateInterval) {
     lastMotorUpdate = millis();

     // Рухаємо currentSpeed до targetSpeed
     if (currentSpeed < targetSpeed) {
      currentSpeed += accelerationRate;
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
      } else if (currentSpeed > targetSpeed) {
      currentSpeed = targetSpeed;  // Миттєве зменшення

     }
      int pwmValue = map(currentSpeed, 0, 100, 0, 255);
     if (forwardflag == 1){
      ledcWrite(PIN_MOTOR_IN1, pwmValue);
      ledcWrite(PIN_MOTOR_IN2, 0);
     }
     if (reverseflag == 1){
      ledcWrite(PIN_MOTOR_IN1, 0);
      ledcWrite(PIN_MOTOR_IN2, pwmValue);
     }}}
//
//
void handleWebMotorControl() {
  // 🔧 Тут згодом реалізації логіки веб-керування двигуном
    // ... код обробки команди ...
  lastWebCommandTime = millis(); // Оновлюємо час при кожній дії
 }



//

//------ВЕБ----------
// === Обробник подій WebSocket ===
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    // Оновлення часу останнього отримання команди та прапорця веб‑керування
    lastWebCommandTime = millis();
    motorControlIntercepted = true;
    
    // Збираємо отримані дані у рядок
    String msg = "";
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    DEBUG_PRINT("Отримано WS повідомлення: ");
    DEBUG_PRINTLN(msg);
    
    // Розбір JSON-команди (припустимо, клієнт надсилає JSON-рядок)
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (!error) {
      // Оновлюємо дані серво
      currentServoAngle = doc["servoX"] | currentServoAngle;
      
      // Отримуємо значення для мотору (наприклад, для керування швидкістю)
      int motorSpeed = doc["servoY"] | 0;  // якщо ключ відсутній, залишаємо 0
      
      // Оновлюємо прапорець керування
      motorControlIntercepted = doc["intercepted"] | false;
      
      // Обробка режиму – якщо отримано режим, обробляємо логіку переходу:
      String modeParam = doc["mode"] | "SERVO_SOFT";
      if (modeParam == "SERVO_HOLD_ZERO") {
        if (servoMode != SERVO_HOLD_ZERO) {
          previousServoMode = servoMode;  // зберігаємо попередній режим
        }
        servoMode = SERVO_HOLD_ZERO;
      } else if (modeParam == "SERVO_HARD" || modeParam == "SERVO_SOFT") {
        // Якщо зараз знаходилися в режимі HOLD_ZERO, відновлюємо попередній режим
        if (servoMode == SERVO_HOLD_ZERO) {
          servoMode = previousServoMode;
        } else {
          servoMode = (modeParam == "SERVO_HARD") ? SERVO_HARD : SERVO_SOFT;
        }
      }
      
      // Оновлюємо стан світла
      headlightOn = doc["light"] | false;
      digitalWrite(PIN_HEADLIGHT, headlightOn ? HIGH : LOW);
      
      // Обробку мотору (MotorForward/MotorReverse) можна додати за потребою,
      // використовуючи значення motorSpeed.
      
    } else {
      DEBUG_PRINT("Помилка розбору JSON: ");
      DEBUG_PRINTLN(error.f_str());
    }
  }
}

// === HTTP-обробник головної сторінки (WebSocket використовуємо замість HTTP update) ===
void handleWEB(AsyncWebServerRequest *request) {
  String page = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  page += "<style>";
  page += "body { font-family: sans-serif; text-align: center; padding: 10px; }";
  page += "input[type=range] { width: 90%; }";
  page += "button { width: 90%; padding: 10px; margin: 5px; font-size: 16px; }";
  page += "</style></head><body>";

  // Відображення поточного режиму серво та кута
  page += "<h2>Режим серво: " + String(servoMode) + "</h2>";
  page += "<p>Кут Руля: <span id='angle'>" + String(currentServoAngle) + "</span></p>";

  // Елемент для керування (джойстик)
  page += "<h3>Джойстик</h3>";
  page += "X: <input type='range' id='x' min='" + String(servoMinAngle) + "' max='" + String(servoMaxAngle) + "' value='" + String(neutralServoAngle) + "'><br>";
  page += "Y: <input type='range' id='y' min='-100' max='100' value='0'><br>";

  // Кнопки управління
  page += "<h3>Управління</h3>";
  page += "<button onclick='toggleControl()' id='btnControl'>Перехопити управління</button>";
  page += "<button onclick='toggleHold()' id='btnHold'>Утримувати 0</button>";
  page += "<button onclick='toggleLight()' id='btnLight'>Світло</button>";
  page += "<button onclick='toggleMode()' id='btnMode'>Тумблер</button>";

  // Кнопка для сторінки налаштувань (HTTP-обробник налаштувань залишається незмінним)
  page += "<br><br><a href='/settings'><button>Налаштування ⚙️</button></a>";

  // JavaScript-код: встановлення WebSocket‑з’єднання та відправка команд
  page += "<script>";
  page += "let intercepted = false, mode = 'SERVO_SOFT', light = false;";

  page += "let socket = new WebSocket('ws://' + window.location.hostname + '/ws');";
  page += "socket.onopen = function(){ console.log('WebSocket з’єднання встановлено'); };";
  page += "socket.onmessage = function(event){ console.log('Отримано WS повідомлення: ' + event.data); };";
  page += "socket.onerror = function(error){ console.error('WS помилка', error); };";
  page += "socket.onclose = function(){ console.log('WebSocket з’єднання закрито'); };";

  // Функція для відправки команд через WebSocket у форматі JSON
  page += "function send() {";
  page += "  let x = document.getElementById('x').value;";
  page += "  let y = document.getElementById('y').value;";
  page += "  document.getElementById('angle').textContent = x;";
  page += "  let cmd = { servoX: x, servoY: y, intercepted: intercepted, mode: mode, light: light };";
  page += "  socket.send(JSON.stringify(cmd));";
  page += "}";
  
  // При зміні значень слайдерів викликаємо send()
  page += "document.getElementById('x').oninput = send;";
  page += "document.getElementById('y').oninput = send;";

  // Кнопкові функції
  page += "function toggleControl() { intercepted = !intercepted; ";
  page += "  document.getElementById('btnControl').innerText = intercepted ? 'Відняти управління' : 'Перехопити управління'; send(); }";
  page += "function toggleHold() { mode = (mode === 'SERVO_HOLD_ZERO') ? 'SERVO_SOFT' : 'SERVO_HOLD_ZERO'; send(); }";
  page += "function toggleLight() { light = !light; send(); }";
  page += "function toggleMode() { mode = (mode === 'SERVO_HARD') ? 'SERVO_SOFT' : 'SERVO_HARD'; send(); }";
  page += "</script>";

  page += "</body></html>";
  request->send(200, "text/html; charset=UTF-8", page);
 }

//
void handleSettings(AsyncWebServerRequest *request) {
  String page = "<h1>Налаштування</h1>";
  //page += "<form action='/save' method='GET'>";
  
      // Поля для введення значень змінних
  page += "Настройка серво: <input name='servang' type='number' step='0.1' value='" + String(currentServoAngle, 1) + "'><br>";
  //page += "иорор: <input name='пумпурум type='number' step='0.1' value='" + String(hysteresis, 1) + "'><br>";
  //page += "роорпм: <input name='пампарам' type='number' value='" + String(sleepMinutes) + "'><br>";
  
  page += "<input type='submit' value='Зберегти'>";
  page += "</form><br>";
  page += "<a href='/'>← Назад</a>";

  request->send(200, "text/html; charset=UTF-8", page); // Відправка сторінки користувачеві
 }
//
void handleSave(AsyncWebServerRequest *request) {
  DEBUG_PRINTLN("Функція /save викликана");
     // Перевіряємо, чи є в запиті нові значення для змінних
     //заготовка. код поки з іншого скетчу
  if (request->hasArg("servang")) {
  // NEW_targetTemp = server.arg("targetTemp").toFloat();  // Отримуємо нове значення для targetTemp
  currentServoAngle = request->arg("servang").toInt();
  }
  //if (server.hasArg("hysteresis")) {
  //  NEW_hysteresis = server.arg("hysteresis").toFloat();  // Отримуємо нове значення для hysteresis
  //}
  //if (server.hasArg("sleepMinutes")) {
  //  NEW_sleepMinutes = server.arg("sleepMinutes").toInt();  // Отримуємо нове значення для sleepMinutes
  moveServoTo(); // Ваша функція для руху серво
      DEBUG_PRINT("Новий кут серво: ");
      DEBUG_PRINTLN(currentServoAngle);
  DEBUG_PRINTLN("Функція веб /зберегти/ спрацьовує ");
  request->send(200, "text/html; charset=UTF-8", "<script>window.location='/';</script>");
  }
//
//
//
void setup() {
    delay(1000);
 #ifdef DEBUG_SERIAL
    Serial.begin(115200);
    Serial.println("DEBUG режим активовано. Вводь змінні у форматі: name = value");
 #endif
 //педалі, кнопки
  pinMode(PIN_SWITCH1, INPUT_PULLUP);
  pinMode(PIN_SWITCH2, INPUT_PULLUP);
  pinMode(PIN_PEDAL,   INPUT_PULLUP);
  pedalStableState = HIGH;
  switch1StableState = HIGH;
  switch2StableState = HIGH;

 //вмикачі всякого
  pinMode(PIN_SERVO_POWER, OUTPUT);
  pinMode(PIN_HEADLIGHT,   OUTPUT);
  
  //pinMode(PIN_MOTOR_EN,    OUTPUT);
  
 // задаём настройки ШИМ-канала мотора:    
  ledcAttach(PIN_MOTOR_IN1,  PWM_FREQ, PWM_RESOLUTION);      //пін для каналу 1(рух вперед)
  ledcAttach(PIN_MOTOR_IN2,  PWM_FREQ, PWM_RESOLUTION);      //пін для каналу 2(рух вперед)
  //digitalWrite(PIN_MOTOR_EN, LOW);

 //настройки ШИМ-канала серво: 
  steeringServo.attach(PIN_SERVO_PWM, 500, 2500); // 500 — мінімальна ширина імпульсу (0° або крайній лівий поворот)
                                                  // 2500 — максимальна ширина (180° або крайній правий поворот)

 //всяке - початковий стан                                                   
  currentServoAngle=neutralServoAngle;
  moveServoTo();     //рух серви в центр
  digitalWrite(PIN_SERVO_POWER, LOW); //
  digitalWrite(PIN_HEADLIGHT, LOW);
  stopMotor();

    // Налаштування WebSocket: реєструємо обробник подій
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Налаштувати веб-сервер (спочатку простий)
  server.on("/", HTTP_GET, handleWEB);               // Головна сторінка
  server.on("/settings", HTTP_GET, handleSettings);  // Сторінка з формою налаштувань
  server.on("/save", HTTP_GET, handleSave);          // Обробник форми — зберігає налаштування
  
  // Підключення до Wi-Fi
WiFi.begin(ssid, password);
DEBUG_PRINT("Підключення до ");
DEBUG_PRINTLN(ssid);

// Очікування з'єднання
int attempts = 0;
while (WiFi.status() != WL_CONNECTED && attempts < 20) {
  delay(500);
  DEBUG_PRINT(".");
  attempts++;
}

// Перевірка результату
if (WiFi.status() == WL_CONNECTED) {
  DEBUG_PRINTLN("\nПідключено! IP адреса: ");
  DEBUG_PRINTLN(WiFi.localIP());

  server.begin();  // Запуск сервера (тільки після підключення до Wi-Fi!)
    DEBUG_PRINTLN("Веб-сервер запущено");
  } else {
    DEBUG_PRINTLN("\nНе вдалося підключитися!");
  }


  DEBUG_PRINTLN("Setup .... ок!");
 }
//
void loop() {
  handleWebServoControl();  // Сервопривід завжди керується через веб
  updateServo(); //вимикає серву після повороту, якщо потрібно

  unsigned long currentMillis = millis();
   
    #ifdef DEBUG_SERIAL
    handleSerialInput();
    #endif

    if (motorControlIntercepted) {
    if (currentMillis - lastWebCommandTime > webControlTimeout) {
      motorControlIntercepted = false;  // Автоматичне повернення до локального керування
      stopMotor();                      // Зупинити двигун при втраті керування
      servoMode = SERVO_SOFT;          // Повернення до м’якого режиму
      DEBUG_PRINTLN("Втрата зв’язку згідно з WebSocket, режим = SERVO_SOFT");
     } else {
     handleWebMotorControl();          // Веб-керування двигуном
     }
     }
 //
  if (!motorControlIntercepted) {
   servoMode = SERVO_SOFT; // М’який режим і локальне керування
   // Дребезг кнопок, читання кнопок  
   bool pedalInput = digitalRead(PIN_PEDAL);  // Зчитуємо стан педалі
   bool switch1Input = digitalRead(PIN_SWITCH1);  // Зчитуємо стан першого перемикача
   bool switch2Input = digitalRead(PIN_SWITCH2);  // Зчитуємо стан другого перемикача
  //
  // Якщо стан будь-якої кнопки змінився і пройшло достатньо часу після останньої зміни (debounce), 
    if ((pedalInput != pedalStableState || 
         switch1Input != switch1StableState || 
         switch2Input != switch2StableState) && (millis() - lastDebounceTime) > debounceDelay) {
         //оновлюємо стабільні стани
         pedalStableState = pedalInput;  // Зберігаємо стабільний стан педалі
         switch1StableState = switch1Input;  // Зберігаємо стабільний стан першого перемикача
         switch2StableState = switch2Input;  // Зберігаємо стабільний стан другого перемикача
         // виводимо відладку в серіал
         DEBUG_PRINT("Педаль: "); DEBUG_PRINT(pedalStableState == LOW ? "Нажата   " : "Відпущена");
         DEBUG_PRINT(", Switch1: "); DEBUG_PRINT(switch1StableState == LOW ? "вкл вперед    " : "назад вимкнено");
         DEBUG_PRINT(", Switch2: "); DEBUG_PRINTLN(switch2StableState == LOW ? "вкл назад      " : "вперед вимкнено");
         lastDebounceTime = millis();  // Оновлюємо час останньої зміни
         }

  if (pedalStableState == LOW && switch1StableState == LOW) { 
    targetSpeed = 100; 
    MotorForward();
    DEBUG_PRINTLN("ВПЕРЕД!");
 } else if (pedalStableState == LOW && switch2StableState == LOW) { 
    targetSpeed = 95;
    MotorReverse();
    DEBUG_PRINTLN("НАЗАД");
 } else {
    stopMotor();
    }

    MotorUpdate();
 
 
 // unsigned long now = millis();
 // if (now - previousMillis >= 100) {
//  previousMillis = now;
 // }
}
}