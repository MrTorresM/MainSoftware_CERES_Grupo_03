#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>  // Para WebSocket

// ===== Configuración de red Wi-Fi =====
const char* ssid     = "Pips";       // Reemplaza por tu SSID
const char* password = "xdxdxdxd";   // Reemplaza por tu contraseña

WebServer server(80);
WebSocketsServer webSocket(81);      // WebSocket en puerto 81

// ===== Pines para DRV8833 (motores A y B) =====
static const int AIN1 = 2;
static const int AIN2 = 4;
static const int BIN1 = 16;
static const int BIN2 = 17;

// ===== Pines para servos de rotación continua =====
static const int SERVO_A_PIN = 18;
static const int SERVO_B_PIN = 19;

Servo servoA;
Servo servoB;

// ===== Pines sensor de color TCS3200 =====
#define S0            32
#define S1            33
#define S2            25
#define S3            26
#define sensorSalida  34

// ===== Pines sensor ultrasónico HC-SR04 =====
#define TRIG_PIN 14
#define ECHO_PIN 27

// ===== Pines divisor resistivo / ADC =====
const int sensorPin = 39;       // ADC 12-bits
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
float maxvoltage = 7.4;

// ===== Magnetómetro HMC5883L/HMC5983 (I²C) =====
#define HMC_ADDR        0x1E
#define REG_CONF_A      0x00
#define REG_CONF_B      0x01
#define REG_MODE        0x02
#define REG_OUT_X_MSB   0x03
const float DECLINATION_ANGLE = -8.33f;  // Bogotá ≈ –8.33°
int16_t rawX, rawY, rawZ;
int16_t minX =  32767, maxX = -32768;
int16_t minY =  32767, maxY = -32768;
float   offsetX = 0, offsetY = 0;

// ===== Variables para lectura de colores =====
int Rojo_Frec  = 0;
int Verde_Frec = 0;
int Azul_Frec  = 0;

// ===== Prototipos =====
void handlePost();
void handleSensors();
void readRawMag();
void sendSensorData();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(115200);
  delay(500);

  // ---- Configurar pines de motores como salida ----
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pararMotores();

  // ---- Configurar servos ----
  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);
  pararServos();

  // ---- Configurar TCS3200 ----
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorSalida, INPUT);
  digitalWrite(S0, HIGH); // escala 20 %
  digitalWrite(S1, LOW);

  // ---- Configurar HC-SR04 ----
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ---- Configurar ADC divisor resistivo ----
  analogReadResolution(12);
  analogSetPinAttenuation(sensorPin, ADC_11db);
  for (int i = 0; i < numReadings; i++) readings[i] = 0;
  total = 0;
  readIndex = 0;

  // ---- Configurar I²C Magnetómetro ----
  Wire.begin(21, 22);
  Wire.setClock(400000UL);
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_CONF_A); Wire.write(0x70); Wire.endTransmission();
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_CONF_B); Wire.write(0xA0); Wire.endTransmission();

  // ---- Calibración de magnetómetro (10 s) ----
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

  // ---- Conectar a Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Conectando a Wi-Fi "); Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200); Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado. IP: "); Serial.println(WiFi.localIP());

  // ---- Rutas HTTP ----
  server.on("/post", HTTP_POST, handlePost);
  server.on("/sensors", HTTP_GET, handleSensors);
  server.begin();
  Serial.println("Servidor HTTP iniciado. Rutas: POST /post, GET /sensors");

  // ---- Iniciar WebSocket ----
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("Servidor WebSocket iniciado en puerto 81");
}

void loop() {
  server.handleClient();
  webSocket.loop();
  sendSensorData();  // Enviar mismo JSON por WebSocket cada 200ms
  delay(200);
}

// ===== Funciones de control de motores =====
void pararMotores() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
void motoresAdelante() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}
void motoresAtras() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}
void motoresGirarDerecha() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);  digitalWrite(BIN2, LOW);
}
void motoresGirarIzquierda() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

// ===== Funciones de control de servos =====
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

// ===== Handler de POST: recibe comandos desde Python =====
void handlePost() {
  if (!server.hasArg("plain")) {
    Serial.println("ERROR: no se recibió mensaje");
    server.send(400, "text/plain", "ERROR");
    return;
  }
  String msg = server.arg("plain");
  msg.trim();
  if (msg.length() < 3 || msg.charAt(1) != ':') {
    Serial.print("Formato inválido: "); Serial.println(msg);
    server.send(400, "text/plain", "ERROR");
    return;
  }
  char stick = msg.charAt(0);         // 'L' o 'R'
  String instr = msg.substring(2);
  if (stick == 'L') {
    if      (instr == "adelante")       motoresAdelante();
    else if (instr == "atras")          motoresAtras();
    else if (instr == "derecha")        motoresGirarDerecha();
    else if (instr == "izquierda")      motoresGirarIzquierda();
    else if (instr == "parar")          pararMotores();
  } else if (stick == 'R') {
    if      (instr == "derecha")        servoAAdelante();
    else if (instr == "izquierda")      servoAtras();
    else if (instr == "adelante")       servoBAdelante();
    else if (instr == "atras")          servoBAtas();
    else if (instr == "parar")          pararServos();
  }
  server.send(200, "text/plain", "OK");
}

// ===== Handler de GET /sensors: devuelve JSON con lecturas =====
void handleSensors() {
  // --- 1) Color TCS3200 ---
  digitalWrite(S2, LOW);  digitalWrite(S3, LOW);  delay(100);
  Rojo_Frec = pulseIn(sensorSalida, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); delay(100);
  Verde_Frec= pulseIn(sensorSalida, LOW);
  digitalWrite(S2, LOW);  digitalWrite(S3, HIGH); delay(100);
  Azul_Frec = pulseIn(sensorSalida, LOW);

  String clasuracionColor = "";
  if      (Rojo_Frec <  85 && Verde_Frec >  80 && Azul_Frec  > 80) clasuracionColor = "ROJO";
  else if (Rojo_Frec >  80 && Verde_Frec > 100 && Azul_Frec  < 90) clasuracionColor = "AZUL";
  else if (Rojo_Frec > 185 && Verde_Frec < 140 && Azul_Frec < 130) clasuracionColor = "VERDE";
  else if (Rojo_Frec <  35 && Verde_Frec <  35 && Azul_Frec  < 35) clasuracionColor = "BLANCO";
  else if (Rojo_Frec > 130 && Verde_Frec > 200 && Azul_Frec >150) clasuracionColor = "NEGRO";

  // --- 2) Ultrasonido HC-SR04 ---
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  float distancia_cm;
  if (dur == 0) distancia_cm = -1.0; else distancia_cm = dur * 0.0343f / 2.0f;

  // --- 3) ADC divisor resistivo ---
  total -= readings[readIndex];
  readings[readIndex] = analogRead(sensorPin);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / (float)numReadings;
  float voltageADC = (average / (float)adcResolution) * referenceVoltage;
  voltageADC *= calibrationFactor;
  actualVoltage = (voltageADC * ((R1 + R2) / R2));

  // --- 4) Magnetómetro HMC5883L/HMC5983 ---
  readRawMag();
  float x = rawX - offsetX;
  float y = rawY - offsetY;
  float headingHTTP = atan2(y, x) * 180.0f / PI;
  if (headingHTTP < 0) headingHTTP += 360;
  headingHTTP += DECLINATION_ANGLE;
  if (headingHTTP < 0) headingHTTP += 360; else if (headingHTTP >= 360) headingHTTP -= 360;

  StaticJsonDocument<512> doc;
  JsonObject jColor = doc.createNestedObject("color");
  jColor["R"] = Rojo_Frec;
  jColor["G"] = Verde_Frec;
  jColor["B"] = Azul_Frec;
  jColor["clasificacion"] = clasuracionColor;
  doc["distancia_cm"] = (distancia_cm < 0 ? String("Fuera de rango") : String(distancia_cm, 1));
  doc["voltaje"] = String(actualVoltage, 3);
  JsonObject jMag = doc.createNestedObject("magnetometro");
  jMag["rawX"] = rawX;
  jMag["rawY"] = rawY;
  jMag["rawZ"] = rawZ;
  jMag["heading°"] = String(headingHTTP, 1);

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// ===== Funciones de WebSocket =====
void sendSensorData() {
  // Usar la lógica exacta proporcionada para WebSocket
  // --- Leer Sensores ---
  // Ultrasonido
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  float distanceCM = (dur == 0) ? -1.0f : dur * 0.0343f / 2.0f;

  // Voltaje y porcentaje
  int porcentajeBateria = constrain((actualVoltage / maxvoltage) * 100.0f, 0, 100);

  StaticJsonDocument<256> doc;
  JsonObject obstaculos = doc.createNestedObject("obstaculos");
  obstaculos["us"] = distanceCM;
  obstaculos["ir"] = distanceCM < 20 && distanceCM >= 0 ? "Obstáculo" : "Libre";
  doc["voltaje"] = actualVoltage;
  doc["bateria"] = porcentajeBateria;

  // Orientación cardinal
  float headingWS = atan2((rawY - offsetY), (rawX - offsetX)) * 180.0f / PI;
  if (headingWS < 0) headingWS += 360.0f;
  headingWS += DECLINATION_ANGLE;
  if (headingWS < 0) headingWS += 360.0f; else if (headingWS >= 360.0f) headingWS -= 360.0f;
  const char* dir;
  if      (headingWS >= 315 || headingWS < 45)   dir = "N";
  else if (headingWS >= 45  && headingWS < 135)  dir = "E";
  else if (headingWS >= 135 && headingWS < 225)  dir = "S";
  else                                           dir = "O";
  //doc["orientacion"] = dir;
  doc["orientacion"] = headingWS;

  // Color por frecuencia
  int minFrec = min(Rojo_Frec, min(Verde_Frec, Azul_Frec));

const char* colorNombre;
const char* rgbColor;

int above400 = 0;
if (Rojo_Frec > 400) above400++;
if (Verde_Frec > 400) above400++;
if (Azul_Frec > 400) above400++;

if (Rojo_Frec < 100 && Verde_Frec < 100 && Azul_Frec < 100) {
  colorNombre = "Blanco";
  rgbColor = "rgb(255,255,255)";
} else if (above400 >= 2) {
  colorNombre = "Negro";
  rgbColor = "rgb(0,0,0)";
} else {
  colorNombre = minFrec == Rojo_Frec  ? "Rojo"  :   
                minFrec == Verde_Frec ? "Verde" : "Azul";

  rgbColor = minFrec == Rojo_Frec  ? "rgb(255,0,0)" :   
             minFrec == Verde_Frec ? "rgb(0,255,0)" : "rgb(0,0,255)";
}


  doc["colorNombre"] = colorNombre;
  doc["colorRGB"]    = rgbColor;

  String out;
  serializeJson(doc, out);
  webSocket.broadcastTXT(out);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("Cliente WS [%u] conectado\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("Cliente WS [%u] desconectado\n", num);
      break;
    case WStype_TEXT:
      // No se procesa texto entrante
      break;
  }
}

// ===== Función para leer ejes crudos del magnetómetro =====
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
  if (rawX > 0x7FFF) rawX -= 0x10000;
  if (rawY > 0x7FFF) rawY -= 0x10000;
  if (rawZ > 0x7FFF) rawZ -= 0x10000;
}
