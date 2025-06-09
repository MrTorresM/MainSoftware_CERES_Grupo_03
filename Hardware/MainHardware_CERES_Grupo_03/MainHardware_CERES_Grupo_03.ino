/***********************************************
 * Rover Control System
 * Comunicación: WebSocket con JSON
 * Sensores: Ultrasonido, Infrarrojo, Color, Voltaje, Magnetómetro
 * Actuadores: Motores DC y Servomotores
 * @authors CERES - Grupo 03
 ***********************************************/

// ===== INCLUDES ==================================
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>  // ← WebSocket

// ===== CONFIGURACIÓN Wi-Fi ========================
const char* ssid     = "CASATOPA";
const char* password = "madadaniela2009";

WebSocketsServer webSocket(81);      // ← WebSocket en puerto 81

// ===== PINES DRV8833 (Motores DC) =================
static const int AIN1 = 2;
static const int AIN2 = 4;
static const int BIN1 = 16;
static const int BIN2 = 17;

// ===== PINES SERVOS ===============================
static const int SERVO_A_PIN = 18;
static const int SERVO_B_PIN = 19;

Servo servoA;
Servo servoB;

// ===== SENSOR DE COLOR ============================
#define S0            32
#define S1            33
#define S2            25
#define S3            26
#define sensorSalida  34

// ===== SENSOR ULTRASÓNICO =========================
#define TRIG_PIN 14
#define ECHO_PIN 27
float distanceCM = 0;

// ===== SENSOR DE VOLTAJE ==========================
const int sensorPin = 39;
const float R1 = 30000.0f;
const float R2 =  7500.0f;
float calibrationFactor = 1.133f;
const int   adcResolution   = 4095;
const float referenceVoltage = 3.3f;
const int numReadings = 10;
int   readings[numReadings];
int   readIndex = 0;
long  total     = 0;
float average   = 0.0f;
float actualVoltage = 0.0f;

// ===== MAGNETÓMETRO ===============================
#define HMC_ADDR        0x1E
#define REG_CONF_A      0x00
#define REG_CONF_B      0x01
#define REG_MODE        0x02
#define REG_OUT_X_MSB   0x03
const float DECLINATION_ANGLE = -8.33f;
int16_t rawX, rawY, rawZ;
int16_t minX =  32767, maxX = -32768;
int16_t minY =  32767, maxY = -32768;
float   offsetX = 0, offsetY = 0;

// ===== VARIABLES DE COLOR =========================
int Rojo_Frec  = 0;
int Verde_Frec = 0;
int Azul_Frec  = 0;

// ===== PROTOTIPOS =================================
void readRawMag();
void readSensors();
void sendSensorData();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

//========================================================
// SETUP
//========================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Conexion wifi temporal

  /*
  Serial.println("Iniciando...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("IP local: ");
  Serial.println(WiFi.localIP());
  
  */



  // ---- Motores y Servos ----
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pararMotores();
  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);
  pararServos();

  // ---- Sensor de Color ----
  pinMode(S0, OUTPUT); digitalWrite(S0, HIGH);
  pinMode(S1, OUTPUT); digitalWrite(S1, LOW);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorSalida, INPUT);

  // ---- Sensor Ultrasónico ----
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  // ---- Voltaje ----
  analogReadResolution(12);
  analogSetPinAttenuation(sensorPin, ADC_11db);
  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  // ---- I²C y magnetómetro ----
  Wire.begin(21, 22);
  Wire.setClock(400000UL);
  Wire.beginTransmission(HMC_ADDR); Wire.write(REG_CONF_A); Wire.write(0x70); Wire.endTransmission();
  Wire.beginTransmission(HMC_ADDR); Wire.write(REG_CONF_B); Wire.write(0xA0); Wire.endTransmission();

  // ---- Calibración magnetómetro ----
  Serial.println("\n> Calibración magnetómetro: gira el módulo 360° durante 10 s");
  unsigned long start = millis();
  while (millis() - start < 10000) {
    readRawMag();
    minX = min(minX, rawX); maxX = max(maxX, rawX);
    minY = min(minY, rawY); maxY = max(maxY, rawY);
    delay(100);
  }
  offsetX = (maxX + minX) * 0.5f;
  offsetY = (maxY + minY) * 0.5f;
  Serial.print("> Offsets X,Y = "); Serial.print(offsetX); Serial.print(" , "); Serial.println(offsetY);
  Serial.println("> Calibración finalizada\n");

  // ---- Conexión Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Conectando a Wi-Fi ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(200); Serial.print(".");
  }
  Serial.println();
  Serial.print("IP local: "); Serial.println(WiFi.localIP());

  // ---- Iniciar WebSocket ----
  webSocket.begin();                                   // Puerto 81
  webSocket.onEvent(webSocketEvent);                  // Evento de WebSocket
  Serial.println("Servidor WebSocket iniciado.");
}

//========================================================
// LOOP
//========================================================

void loop() {
  webSocket.loop();       // Maneja conexiones WebSocket
  sendSensorData();       // Envía datos JSON en tiempo real
  delay(200);             // Frecuencia de actualización (ajustable)
}

// ========================================================
//  FUNCIONES DE COMUNICACIÓN
// ========================================================
/*
 * Envía los datos de sensores en formato JSON a todos los clientes WebSocket conectados.
 */
void sendSensorData() {
  readSensors();

  StaticJsonDocument<256> doc;

  // Enviar obstáculos
  JsonObject obstaculos = doc.createNestedObject("obstaculos");
  obstaculos["us"] = distanceCM;
  obstaculos["ir"] = distanceCM < 20 ? "Obstáculo" : "Libre"; // Ejemplo IR simple

  // Enviar voltaje y porcentaje
  const float maxReachableVoltage = 7.4f; // Ejemplo: batería Li-Ion 2S
  int porcentajeBateria = constrain((actualVoltage / maxReachableVoltage) * 100.0f, 0, 100);
  doc["voltaje"] = actualVoltage;
  doc["bateria"] = porcentajeBateria;

  // Enviar orientación como N, E, S, O
  float heading = atan2((float)(rawY - offsetY), (float)(rawX - offsetX));
  heading = heading * 180 / PI + DECLINATION_ANGLE;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;

  const char* dir;
  if (heading >= 315 || heading < 45) dir = "N";
  else if (heading >= 45 && heading < 135) dir = "E";
  else if (heading >= 135 && heading < 225) dir = "S";
  else dir = "O";

  doc["orientacion"] = dir;

  // Detección de color por frecuencia más baja
  int minFrec = min(Rojo_Frec, min(Verde_Frec, Azul_Frec));
  const char* colorNombre = "Desconocido";
  const char* rgbColor = "rgb(255,255,255)";
  if (minFrec == Rojo_Frec) {
    colorNombre = "Rojo";
    rgbColor = "rgb(255,0,0)";
  } else if (minFrec == Verde_Frec) {
    colorNombre = "Verde";
    rgbColor = "rgb(0,255,0)";
  } else if (minFrec == Azul_Frec) {
    colorNombre = "Azul";
    rgbColor = "rgb(0,0,255)";
  }

  doc["colorNombre"] = colorNombre;
  doc["colorRGB"] = rgbColor;

  String output;
  serializeJson(doc, output);
  webSocket.broadcastTXT(output);
}


/*
 * Maneja eventos WebSocket: conexión, mensaje, desconexión.
 */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("Cliente [%u] conectado\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("Cliente [%u] desconectado\n", num);
      break;
    case WStype_TEXT:
      Serial.printf("Mensaje [%u]: %s\n", num, payload);
      // Aquí podrías leer comandos en JSON entrantes para controlar motores
      break;
  }
}

// ========================================================
//  FUNCIONES DE LECTURA
// ========================================================


void readSensors() {
  // --- Ultrasonido ---
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  distanceCM = duracion * 0.034 / 2;

  // --- Voltaje (promediado) ---
  total -= readings[readIndex];
  readings[readIndex] = analogRead(sensorPin);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / (float)numReadings;
  float vOUT = average * referenceVoltage / adcResolution;
  actualVoltage = vOUT / (R2 / (R1 + R2)) * calibrationFactor;

  // --- Magnetómetro ---
  readRawMag();

  // --- Color ---
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); delay(100);
  Rojo_Frec = pulseIn(sensorSalida, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); delay(100);
  Verde_Frec = pulseIn(sensorSalida, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); delay(100);
  Azul_Frec = pulseIn(sensorSalida, LOW);
}

// ========================================================
//  LECTURA DE MAGNETÓMETRO
// ========================================================
void readRawMag() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_MODE); Wire.write(0x01); Wire.endTransmission();
  delay(6);
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC_ADDR, (uint8_t)6);
  rawX = (Wire.read() << 8) | Wire.read();
  rawZ = (Wire.read() << 8) | Wire.read();
  rawY = (Wire.read() << 8) | Wire.read();
}

// ========================================================
//  FUNCIONES DE MOTORES Y SERVOS
// ========================================================
void pararMotores() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
void motoresAdelante() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}
void motoresAtras() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}
void motoresGirarDerecha() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
}
void motoresGirarIzquierda() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}
void pararServos() {
  servoA.writeMicroseconds(1500);
  servoB.writeMicroseconds(1500);
}
void servoAAdelante() {
  servoA.writeMicroseconds(2000);
}
void servoAtras() {
  servoA.writeMicroseconds(1000);
}
void servoBAdelante() {
  servoB.writeMicroseconds(2000);
}
void servoBAtas() {
  servoB.writeMicroseconds(1000);
}