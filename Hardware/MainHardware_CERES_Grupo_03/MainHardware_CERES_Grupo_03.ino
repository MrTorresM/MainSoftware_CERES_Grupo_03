#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ===== Configuración de red Wi-Fi =====
const char* ssid     = "Pips";       // Reemplaza por tu SSID
const char* password = "xdxdxdxd";   // Reemplaza por tu contraseña

WebServer server(80);

// ===== Pines para DRV8833 (motores A y B) =====
static const int AIN1 = 2;
static const int AIN2 = 4;
static const int BIN1 = 16;
static const int BIN2 = 17;

// ===== Pines para servos de rotación continua =====
static const int SERVO_A_PIN = 18;  // servo A
static const int SERVO_B_PIN = 19;  // servo B

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
  digitalWrite(S0, HIGH); // escala 20 % (S0=HIGH, S1=LOW)
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
  // CRA: 8 muestras promedio, 15 Hz, modo normal
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_CONF_A);
  Wire.write(0x70);
  Wire.endTransmission();
  // CRB: ganancia ±5 gauss
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_CONF_B);
  Wire.write(0xA0);
  Wire.endTransmission();

  // Calibración de magnetómetro (10 s)
  Serial.println("\n> Calibración magnetómetro: gira el módulo 360° durante 10 s");
  unsigned long start = millis();
  while (millis() - start < 10000) {
    readRawMag();
    minX = min(minX, rawX);
    maxX = max(maxX, rawX);
    minY = min(minY, rawY);
    maxY = max(maxY, rawY);
    delay(100);
  }
  offsetX = (maxX + minX) * 0.5f;
  offsetY = (maxY + minY) * 0.5f;
  Serial.print("> Offsets X,Y = ");
  Serial.print(offsetX); Serial.print(" , "); Serial.println(offsetY);
  Serial.println("> Calibración finalizada\n");

  // ---- Conectar a Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Conectando a Wi-Fi ");
  Serial.print(ssid);
  Serial.print(" ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Conectado. IP: ");
  Serial.println(WiFi.localIP());

  // ---- Rutas HTTP ----
  server.on("/post", HTTP_POST, handlePost);
  server.on("/sensors", HTTP_GET, handleSensors);
  server.begin();
  Serial.println("Servidor HTTP iniciado. Rutas:");
  Serial.println(" POST /post    ← recibe comandos de joystick");
  Serial.println(" GET /sensors  ← devuelve JSON con lecturas de sensores");
}

void loop() {
  server.handleClient();
}

// ===== Funciones de control de motores =====
void pararMotores() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
void motoresAdelante() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}
void motoresAtras() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}
void motoresGirarDerecha() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}
void motoresGirarIzquierda() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
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
    Serial.print("Formato inválido: ");
    Serial.println(msg);
    server.send(400, "text/plain", "ERROR");
    return;
  }

  char stick = msg.charAt(0);         // 'L' o 'R'
  String instruccion = msg.substring(2);

  if (stick == 'L') {
    // Control de motores (stick izquierdo)
    if      (instruccion == "adelante")       { motoresAdelante();       Serial.println("Motores → Adelante"); }
    else if (instruccion == "atras")           { motoresAtras();          Serial.println("Motores → Atrás"); }
    else if (instruccion == "derecha")         { motoresGirarDerecha();   Serial.println("Motores → Girar Derecha"); }
    else if (instruccion == "izquierda")       { motoresGirarIzquierda(); Serial.println("Motores → Girar Izquierda"); }
    else if (instruccion == "parar")           { pararMotores();          Serial.println("Motores → Parar"); }
    else {
      Serial.print("Motores: instrucción inválida: ");
      Serial.println(instruccion);
    }
  }
  else if (stick == 'R') {
    // Control de servos (stick derecho)
    if      (instruccion == "derecha")         { servoAAdelante();        Serial.println("Servo A → Adelante"); }
    else if (instruccion == "izquierda")       { servoAtras();            Serial.println("Servo A → Atrás"); }
    else if (instruccion == "adelante")        { servoBAdelante();        Serial.println("Servo B → Adelante"); }
    else if (instruccion == "atras")           { servoBAtas();            Serial.println("Servo B → Atrás"); }
    else if (instruccion == "parar")           { pararServos();           Serial.println("Servos → Parar"); }
    else {
      Serial.print("Servos: instrucción inválida: ");
      Serial.println(instruccion);
    }
  }
  else {
    Serial.print("Prefijo inválido: ");
    Serial.println(msg);
  }

  server.send(200, "text/plain", "OK");
}

// ===== Handler de GET /sensors: devuelve JSON con lecturas =====
void handleSensors() {
  // --- 1) Lectura sensor de color TCS3200 ---
  // Rojo
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(100);
  Rojo_Frec = pulseIn(sensorSalida, LOW);
  // Verde
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(100);
  Verde_Frec = pulseIn(sensorSalida, LOW);
  // Azul
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(100);
  Azul_Frec = pulseIn(sensorSalida, LOW);

  String clasificacionColor = "";
  if      (Rojo_Frec <  85 && Verde_Frec >  80 && Azul_Frec  > 80)   clasificacionColor = "ROJO";
  else if (Rojo_Frec >  80 && Verde_Frec > 100 && Azul_Frec  < 90)   clasificacionColor = "AZUL";
  else if (Rojo_Frec > 185 && Verde_Frec < 140 && Azul_Frec < 130)   clasificacionColor = "VERDE";
  else if (Rojo_Frec <  35 && Verde_Frec <  35 && Azul_Frec  < 35)   clasificacionColor = "BLANCO";
  else if (Rojo_Frec > 130 && Verde_Frec > 200 && Azul_Frec >150)    clasificacionColor = "NEGRO";

  // --- 2) Lectura ultrasónico HC-SR04 ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  float distancia_cm;
  if (dur == 0) distancia_cm = -1.0; // fuera de rango
  else          distancia_cm = dur * 0.0343f / 2.0f;

  // --- 3) Lectura divisor resistivo / ADC ---
  total -= readings[readIndex];
  readings[readIndex] = analogRead(sensorPin);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / (float)numReadings;
  float voltageADC = (average / (float)adcResolution) * referenceVoltage;
  voltageADC *= calibrationFactor;
  actualVoltage = voltageADC * ((R1 + R2) / R2);

  // --- 4) Lectura magnetómetro HMC5883L/HMC5983 ---
  readRawMag();
  float x = rawX - offsetX;
  float y = rawY - offsetY;
  float heading = atan2(y, x) * 180.0f / PI;
  if (heading < 0)       heading += 360.0f;
  heading += DECLINATION_ANGLE;
  if (heading < 0)       heading += 360.0f;
  else if (heading >= 360) heading -= 360.0f;

  // --- Construir JSON de respuesta ---
  StaticJsonDocument<512> doc;

  // Color
  JsonObject jColor = doc.createNestedObject("color");
  jColor["R"] = Rojo_Frec;
  jColor["G"] = Verde_Frec;
  jColor["B"] = Azul_Frec;
  jColor["clasificacion"] = clasificacionColor;

  // Distancia
  doc["distancia_cm"] = (distancia_cm < 0 ? String("Fuera de rango") : String(distancia_cm, 1));

  // Voltaje
  doc["voltaje"] = String(actualVoltage, 3);

  // Magnetómetro
  JsonObject jMag = doc.createNestedObject("magnetometro");
  jMag["rawX"] = rawX;
  jMag["rawY"] = rawY;
  jMag["rawZ"] = rawZ;
  jMag["heading°"] = String(heading, 1);

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// ===== Función para leer ejes crudos del magnetómetro =====
void readRawMag() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_MODE);
  Wire.write(0x01);  // Single-measurement mode
  Wire.endTransmission();
  delay(6);

  Wire.beginTransmission(HMC_ADDR);
  Wire.write(REG_OUT_X_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC_ADDR, (uint8_t)6);

  rawX = (Wire.read() << 8) | Wire.read();
  rawZ = (Wire.read() << 8) | Wire.read();
  rawY = (Wire.read() << 8) | Wire.read();

  if (rawX >  0x7FFF) rawX -= 0x10000;
  if (rawY >  0x7FFF) rawY -= 0x10000;
  if (rawZ >  0x7FFF) rawZ -= 0x10000;
}
