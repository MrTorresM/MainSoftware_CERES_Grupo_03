/***********************************************
 * Rover Control System
 * Sensores: Ultrasonido, Infrarrojo, Color, Voltaje
 * Actuadores: Motores DC y Servomotores
 * @authors CERES - Grupo 03
 ***********************************************/

//========================================================
// CONFIGURACIÓN GENERAL
//========================================================

//----------------------------------------------------
// Pines de sensores

// Pines del sensor ultrasonido () - (5V-GND)
const int PIN_ULTRASONIDO_TRIG = 12;
const int PIN_ULTRASONIDO_ECHO = 14;

// Pines del sensor infrarrojo () - (3.3V-GND)
const int PIN_INFRARED = 27;

// Pines del sensor de color FRONTAL () - (3.3V-GND)
const int PIN_TCS_FRONTAL_S0 = 32;
const int PIN_TCS_FRONTAL_S1 = 33;
const int PIN_TCS_FRONTAL_S2 = 25;
const int PIN_TCS_FRONTAL_S3 = 26;
const int PIN_TCS_FRONTAL_OUT = 4;

// Pines del sensor de color del SUELO () - (3.3V-GND)
const int PIN_TCS_SUELO_S0 = 15;
const int PIN_TCS_SUELO_S1 = 2;
const int PIN_TCS_SUELO_S2 = 5;
const int PIN_TCS_SUELO_S3 = 23;
const int PIN_TCS_SUELO_OUT = 13;

// Pines del sensor de voltaje () (GND)
const int PIN_SENSOR_VOLTAGE = 35;

/*
const int PIN_TCS_S0 = 15;
const int PIN_TCS_S1 = 2;
const int PIN_TCS_S2 = 4;
const int PIN_TCS_S3 = 18;
const int PIN_TCS_OUT = 19;
*/

//----------------------------------------------------
// Pines de motores
const int PIN_MOTOR_LEFT_FORWARD = 16;
const int PIN_MOTOR_LEFT_BACKWARD = 17;
const int PIN_MOTOR_RIGHT_FORWARD = 18;
const int PIN_MOTOR_RIGHT_BACKWARD = 19;

//----------------------------------------------------
// Pines de servos
const int PIN_SERVO_ARM = 21;
const int PIN_SERVO_GRIPPER = 22;

//----------------------------------------------------
// Parámetros de sensores
const float DISTANCIA_ALERTA_ULTRASONIDO = 20.0; // centímetros
const int DETECCION_INFRAROJO_UMBRAL = 500; // valor analógico
const float NIVEL_BATERIA_CRITICO = 3.3; // voltios

//----------------------------------------------------
// Definición de comandos de movimiento
const int MOVER_ADELANTE = 1;
const int MOVER_ATRAS = 2;
const int GIRAR_IZQUIERDA = 3;
const int GIRAR_DERECHA = 4;

//========================================================
// Librerías
//========================================================

#include <ESP32Servo.h>

//========================================================
// Objetos
//========================================================

Servo servoBrazo;
Servo servoPinza;

//========================================================
// SETUP
//========================================================

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines
  pinMode(PIN_ULTRASONIDO_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIDO_ECHO, INPUT);
  pinMode(PIN_INFRARED, INPUT);
  pinMode(PIN_SENSOR_VOLTAGE, INPUT);

  pinMode(PIN_MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_BACKWARD, OUTPUT);

  servoBrazo.attach(PIN_SERVO_ARM);
  servoPinza.attach(PIN_SERVO_GRIPPER);
}

//========================================================
// BUCLE PRINCIPAL
//========================================================

void loop() {

  //---------------------------------------
  // Sensor ultrasonido

  
  float distancia = leerUltrasonido();
  if (distancia <= DISTANCIA_ALERTA_ULTRASONIDO) {
    Serial.println("OBSTACULO DETECTADO POR ULTRASONIDO");
  }

  //---------------------------------------
  // Sensor infrarrojo

  if (leerInfrarrojo()) {
    Serial.println("OBSTACULO DETECTADO POR INFRARROJO");
  }

  

  //---------------------------------------
  // Sensores de color

  //---- Sensor del suelo
  //leerColorSuelo(); 

  //---- Sensor de la pinza
  //leerColorFrontal();

  //---------------------------------------
  // Sensor de voltaje
  float voltaje = leerVoltaje();
  Serial.print("Voltaje actual: ");
  Serial.println(voltaje);

  if (voltaje <= NIVEL_BATERIA_CRITICO) {
    Serial.println("NIVEL DE BATERIA CRITICO");
  }

  //---------------------------------------
  // Control de motores
  
  //int comandoRecibido = MOVER_ADELANTE;

  //-- Ejecuta comando recibido
  //controlarMovimiento(comandoRecibido);
  
  delay(1000); 
}

//========================================================
// FUNCIONES DE LECTURA Y ENVIO DE DATOS
//========================================================

void leerSerial() {


}

//========================================================
// FUNCIONES DE SENSORES
//========================================================

/**
 * Lee la distancia utilizando el sensor ultrasonido HC-SR04.
 * @return distancia en centímetros.
 */
float leerUltrasonido() {
  digitalWrite(PIN_ULTRASONIDO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIDO_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIDO_TRIG, LOW);

  long duracion = pulseIn(PIN_ULTRASONIDO_ECHO, HIGH);
  float distancia = duracion * 0.034 / 2;
  return distancia;
}

//-------------------------------------------------------------------

/**
 * Detecta obstáculos usando sensor infrarrojo.
 * @return true si hay obstáculo, false si libre.
 */
bool leerInfrarrojo() {
  int valor = analogRead(PIN_INFRARED);
  return valor > DETECCION_INFRAROJO_UMBRAL;
}

//-------------------------------------------------------------------

/**
 * Lee y muestra los valores de color del sensor de suelo (TCS3200).
 */
void leerColorSuelo() {
  // Configura la escala de frecuencia
  pinMode(PIN_TCS_SUELO_S0, OUTPUT);
  pinMode(PIN_TCS_SUELO_S1, OUTPUT);
  digitalWrite(PIN_TCS_SUELO_S0, HIGH);
  digitalWrite(PIN_TCS_SUELO_S1, HIGH);

  int redFrequency, greenFrequency, blueFrequency;

  // Lee rojo
  digitalWrite(PIN_TCS_SUELO_S2, LOW);
  digitalWrite(PIN_TCS_SUELO_S3, LOW);
  redFrequency = pulseIn(PIN_TCS_SUELO_OUT, LOW);

  // Lee verde
  digitalWrite(PIN_TCS_SUELO_S2, HIGH);
  digitalWrite(PIN_TCS_SUELO_S3, HIGH);
  greenFrequency = pulseIn(PIN_TCS_SUELO_OUT, LOW);

  // Lee azul
  digitalWrite(PIN_TCS_SUELO_S2, LOW);
  digitalWrite(PIN_TCS_SUELO_S3, HIGH);
  blueFrequency = pulseIn(PIN_TCS_SUELO_OUT, LOW);

  Serial.print("Color del suelo -> R: ");
  Serial.print(redFrequency);
  Serial.print(" G: ");
  Serial.print(greenFrequency);
  Serial.print(" B: ");
  Serial.println(blueFrequency);
}


//---------

/**
 * Lee y muestra los valores de color del sensor frontal (TCS3200).
 */
void leerColorFrontal() {
  // Configura la escala de frecuencia
  pinMode(PIN_TCS_FRONTAL_S0, OUTPUT);
  pinMode(PIN_TCS_FRONTAL_S1, OUTPUT);
  digitalWrite(PIN_TCS_FRONTAL_S0, HIGH);
  digitalWrite(PIN_TCS_FRONTAL_S1, HIGH);

  int redFrequency, greenFrequency, blueFrequency;

  // Lee rojo
  digitalWrite(PIN_TCS_FRONTAL_S2, LOW);
  digitalWrite(PIN_TCS_FRONTAL_S3, LOW);
  redFrequency = pulseIn(PIN_TCS_SUELO_OUT, LOW);

  // Lee verde
  digitalWrite(PIN_TCS_FRONTAL_S2, HIGH);
  digitalWrite(PIN_TCS_FRONTAL_S3, HIGH);
  greenFrequency = pulseIn(PIN_TCS_SUELO_OUT, LOW);

  // Lee azul
  digitalWrite(PIN_TCS_FRONTAL_S2, LOW);
  digitalWrite(PIN_TCS_FRONTAL_S3, HIGH);
  blueFrequency = pulseIn(PIN_TCS_FRONTAL_OUT, LOW);

  Serial.print("Color del frente -> R: ");
  Serial.print(redFrequency);
  Serial.print(" G: ");
  Serial.print(greenFrequency);
  Serial.print(" B: ");
  Serial.println(blueFrequency);
}

//-------------------------------------------------------------------

/**
 * Lee el voltaje actual de la batería.
 * @return voltaje en voltios.
 */
float leerVoltaje() {
  int rawValue = analogRead(PIN_SENSOR_VOLTAGE);
  float voltage = rawValue * (3.3 / 4095.0); // Ajustar según divisor de voltaje si aplica.
  return voltage;
}

//========================================================
// FUNCIONES DE MOVIMIENTO
//========================================================

/**
 * Controla el movimiento de los motores DC.
 * @param comando: Dirección basada en las constantes predefinidas.
 */
void controlarMovimiento(int comando) {
  switch (comando) {
    case MOVER_ADELANTE:
      digitalWrite(PIN_MOTOR_LEFT_FORWARD, HIGH);
      digitalWrite(PIN_MOTOR_LEFT_BACKWARD, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_FORWARD, HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_BACKWARD, LOW);
      break;
    case MOVER_ATRAS:
      digitalWrite(PIN_MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(PIN_MOTOR_LEFT_BACKWARD, HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_BACKWARD, HIGH);
      break;
    case GIRAR_IZQUIERDA:
      digitalWrite(PIN_MOTOR_LEFT_FORWARD, LOW);
      digitalWrite(PIN_MOTOR_LEFT_BACKWARD, HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_FORWARD, HIGH);
      digitalWrite(PIN_MOTOR_RIGHT_BACKWARD, LOW);
      break;
    case GIRAR_DERECHA:
      digitalWrite(PIN_MOTOR_LEFT_FORWARD, HIGH);
      digitalWrite(PIN_MOTOR_LEFT_BACKWARD, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_FORWARD, LOW);
      digitalWrite(PIN_MOTOR_RIGHT_BACKWARD, HIGH);
      break;
    default:
      detenerMotores();
      break;
  }
}

//-------------------------------------------------------------------

/**
 * Detiene todos los motores.
 */
void detenerMotores() {
  digitalWrite(PIN_MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(PIN_MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_BACKWARD, LOW);
}

//========================================================
// FUNCIONES DEL BRAZO
//========================================================

/**
 * Mueve el brazo robótico a un ángulo dado.
 * @param angulo: ángulo en grados.
 */
void moverBrazo(int angulo) {
  servoBrazo.write(angulo);
}

/**
 * Abre o cierra la pinza.
 * @param angulo: ángulo de apertura/cierre.
 */
void controlarPinza(int angulo) {
  servoPinza.write(angulo);
}

//-------------------------------------------------------------------